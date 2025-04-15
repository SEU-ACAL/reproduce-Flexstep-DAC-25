// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package freechips.rocketchip.rocket
//instcoun checkdone signal || dmem.kill
import chisel3._
import chisel3.util._
import chisel3.withClock
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property
import freechips.rocketchip.scie._
import scala.collection.mutable.ArrayBuffer
import freechips.rocketchip.diplomacy.LazyModule
import org.yaml.snakeyaml.events.Event.ID

import freechips.rocketchip.snapshot._
import org.w3c.dom.css

case class RocketCoreParams(
  bootFreqHz: BigInt = 0,
  useVM: Boolean = true,
  useUser: Boolean = false,
  useSupervisor: Boolean = false,
  useHypervisor: Boolean = false,
  useDebug: Boolean = true,
  useAtomics: Boolean = true,
  useAtomicsOnlyForIO: Boolean = false,
  useCompressed: Boolean = true,
  useRVE: Boolean = false,
  useSCIE: Boolean = false,
  useBitManip: Boolean = false,
  useBitManipCrypto: Boolean = false,
  useCryptoNIST: Boolean = false,
  useCryptoSM: Boolean = false,
  useConditionalZero: Boolean = false,
  nLocalInterrupts: Int = 0,
  useNMI: Boolean = false,
  nBreakpoints: Int = 1,
  useBPWatch: Boolean = false,
  mcontextWidth: Int = 0,
  scontextWidth: Int = 0,
  nPMPs: Int = 8,
  nPerfCounters: Int = 0,
  haveBasicCounters: Boolean = true,
  haveCFlush: Boolean = false,
  misaWritable: Boolean = true,
  nL2TLBEntries: Int = 0,
  nL2TLBWays: Int = 1,
  nPTECacheEntries: Int = 8,
  mtvecInit: Option[BigInt] = Some(BigInt(0)),
  mtvecWritable: Boolean = true,
  fastLoadWord: Boolean = true,
  fastLoadByte: Boolean = false,
  branchPredictionModeCSR: Boolean = false,
  clockGate: Boolean = false,
  mvendorid: Int = 0, // 0 means non-commercial implementation
  mimpid: Int = 0x20181004, // release date in BCD
  mulDiv: Option[MulDivParams] = Some(MulDivParams()),
  fpu: Option[FPUParams] = Some(FPUParams()),
  haveCease: Boolean = true, // non-standard CEASE instruction
  debugROB: Boolean = false // if enabled, uses a C++ debug ROB to generate trace-with-wdata
) extends CoreParams {
  val lgPauseCycles = 5
  val haveFSDirty = false
  val pmpGranularity: Int = if (useHypervisor) 4096 else 4
  val fetchWidth: Int = if (useCompressed) 2 else 1
  //  fetchWidth doubled, but coreInstBytes halved, for RVC:
  val decodeWidth: Int = fetchWidth / (if (useCompressed) 2 else 1)
  val retireWidth: Int = 1
  val instBits: Int = if (useCompressed) 16 else 32
  val lrscCycles: Int = 80 // worst case is 14 mispredicted branches + slop
  val traceHasWdata: Boolean = false // ooo wb, so no wdata in trace
  override val customIsaExt = Option.when(haveCease)("xrocket") // CEASE instruction
  override def minFLen: Int = fpu.map(_.minFLen).getOrElse(32)
  override def customCSRs(implicit p: Parameters) = new RocketCustomCSRs
}

trait HasRocketCoreParameters extends HasCoreParameters {
  lazy val rocketParams: RocketCoreParams = tileParams.core.asInstanceOf[RocketCoreParams]

  val fastLoadWord = rocketParams.fastLoadWord
  val fastLoadByte = rocketParams.fastLoadByte

  val mulDivParams = rocketParams.mulDiv.getOrElse(MulDivParams()) // TODO ask andrew about this

  val usingABLU = usingBitManip || usingBitManipCrypto
  val aluFn = if (usingABLU) new ABLUFN else new ALUFN

  require(!fastLoadByte || fastLoadWord)
  require(!rocketParams.haveFSDirty, "rocket doesn't support setting fs dirty from outside, please disable haveFSDirty")
  require(!(usingABLU && usingConditionalZero), "Zicond is not yet implemented in ABLU")
}

class RocketCustomCSRs(implicit p: Parameters) extends CustomCSRs with HasRocketCoreParameters {
  override def bpmCSR = {
    rocketParams.branchPredictionModeCSR.option(CustomCSR(bpmCSRId, BigInt(1), Some(BigInt(0))))
  }

  private def haveDCache = tileParams.dcache.get.scratch.isEmpty

  override def chickenCSR = {
    val mask = BigInt(
      tileParams.dcache.get.clockGate.toInt << 0 |
      rocketParams.clockGate.toInt << 1 |
      rocketParams.clockGate.toInt << 2 |
      1 << 3 | // disableSpeculativeICacheRefill
      haveDCache.toInt << 9 | // suppressCorruptOnGrantData
      tileParams.icache.get.prefetch.toInt << 17
    )
    Some(CustomCSR(chickenCSRId, mask, Some(mask)))
  }

  def disableICachePrefetch = getOrElse(chickenCSR, _.value(17), true.B)

  def marchid = CustomCSR.constant(CSRs.marchid, BigInt(1))

  def mvendorid = CustomCSR.constant(CSRs.mvendorid, BigInt(rocketParams.mvendorid))

  // mimpid encodes a release version in the form of a BCD-encoded datestamp.
  def mimpid = CustomCSR.constant(CSRs.mimpid, BigInt(rocketParams.mimpid))

  override def decls = super.decls :+ marchid :+ mvendorid :+ mimpid
}

class Rocket(tile: RocketTile)(implicit p: Parameters) extends CoreModule()(p)
    with HasRocketCoreParameters
    with HasCoreIO {
  def nTotalRoCCCSRs = tile.roccCSRs.flatten.size

  //custom Module and Reg define
  val FIFO        = Module(new SyncFIFO(GlobalParams.Data_type, GlobalParams.depth))
  val otmmux      = Module(new otmMux(GlobalParams.Num_Groupcores))
  val mtomux      = Module(new mtoMux(GlobalParams.Num_Groupcores))
  val my_log_unit = Module(new LogUnit()(p))
  // val test_fifo  = Module(new HellaQueue(16)(UInt(32.W)))
  // val test_fifo1 = Module(new HellaQueue(16)(UInt(32.W)))

  // // privilege level and ppn monitor 
  val task_monitor = Wire(new task_monitor())

  //some case need flush at wb stage
  val wb_flush = Wire(new wb_flush_pipe())

  //some signals for copy rf data
  val copy_signals = Wire(new need_copy())

  val custom_reg = RegInit(44.U(32.W))
  val custom_regout = RegInit(1.U(32.W))
  val custom_regin = RegInit(0.U(32.W))
  
  //val counter = RegInit(0.U(32.W))

  val Mcheck_call = WireInit(false.B)//mcore call for starting checking
  val Mchecke_call = WireInit(false.B)//mcore call for ending checking
  val Mcheck_end   = WireInit(false.B)
  val Scheck_end   = WireInit(false.B)

  val Srecode_call           = WireInit(false.B)//score call for recoding context and npc
  val score_apply            = WireInit(false.B)//score call for receving rf
  val start_check            = RegInit(false.B)//indicate the start of all checking 
  val end_check              = RegInit(false.B)//indicate mcore call for ending check and sent rf_data for comparing
  val mcore_checking         = RegInit(false.B)//indicate mcore is between two CPs
  val check_busy             = RegInit(false.B)//indicate score is checking
  

  val score_return_pc = RegInit(0.U(40.W))

  val mcore_check_done        = RegInit(false.B)
  val mcore_check_done_common = RegInit(false.B)
  val score_check_done        = RegInit(false.B)
  val score_comp_done         = RegInit(false.B)

  val ppn_change_count        = RegInit(0.U(32.W))
  val ppn_trace_original      = RegInit(0.U((maxPAddrBits - pgIdxBits).W))
  val ppn_trace               = Reg(UInt((maxPAddrBits - pgIdxBits).W))

  val score_return_rf  = RegInit(false.B)
  val score_check_left = RegInit(0.U(64.W)) 
  val check_instnum_threshold = RegInit(5000.U(64.W))

  val custom_jalr = WireInit(false.B)
  val have_jumped = RegInit(false.B)
  val have_reve   = RegInit(false.B)
  

  val check_mode  = RegInit(true.B)//indicate score is at check mode
  val user_mode = task_monitor.trace_priv === PRV.U.U
  val slave_is_umode = WireInit(false.B)
  
  val jump_pc = RegInit(0.U(64.W))//the pc that score jump for

  val CP_start       = RegInit(false.B)
  val CP_end         = RegInit(false.B)
  val CP_mid_ofchange  = RegInit(false.B)

  //val rdata = WireInit(0.U(256.W))//for slave core to receive data
  val rf_read_data    = RegInit(0.U(64.W))  
  val rf_read_addr    = WireInit(0.U(5.W))

  val rf_ready        = WireInit(false.B)
  val receiving_rf    = WireInit(false.B)//slave is receiving rf
  val rf_sign         = WireInit(false.B)
  val rece_rf_done    = WireInit(false.B)
  val rf_apply_en     = WireInit(false.B)
  val rf_apply_data   = WireInit(0.U(64.W))
  val rf_apply_addr   = WireInit(0.U(5.W))

  val rdata = WireInit(0.U(256.W))
  //mode toggle
  val mode_signe = WireInit(false.B)// end check
  val mode_signs = WireInit(false.B)// start check
  val mode_signc = WireInit(false.B)// task change
  val inst_left  = WireInit(0.U(64.W))

  val pc_sign    = WireInit(false.B)
  val past_sign  = WireInit(false.B)
  //rf data
  val rf_datatype = WireInit(0.U(2.W))
  val widx        = WireInit(13.U(5.W))
  val wrf_data    = WireInit(0.U(64.W))
  val wfprf_data  = WireInit(0.U(64.W))
  //ls data
  val ls_sign  = WireInit(false.B)
  val ls_inst  = WireInit(0.U(32.W))
  val ls_addr  = WireInit(0.U(64.W))
  val ld_value = WireInit(0.U(64.W))  
  val st_value = WireInit(0.U(64.W))

  val rf_counter = Counter(0 until 32, rf_sign && (rf_datatype === 1.U) && FIFO.io.out.ready, ls_sign)
  val rf_count   = rf_counter._1
  val rf_wrap    = rf_counter._2

  val fprf_data = WireInit(0.U(64.W))//receive fp arf
  val fprf_idx = WireInit(0.U(5.W))
  val fp_apply_en = WireInit(false.B)

  val csr_apply_en = WireInit(false.B)

  val fcsr_out = WireInit(0.U(8.W))

  val sc_undone = RegInit(false.B)
  val sc_cond   = RegInit(1.U(64.W))

  val sentdone_butuncheck = RegInit(false.B)

  val imem_req_pc = RegInit(0.U(40.W))

  val timer_counter = RegInit(0.U(32.W))
 
  dontTouch(receiving_rf)
  dontTouch(rece_rf_done)
  dontTouch(fcsr_out)
  dontTouch(fprf_data)
  dontTouch(fp_apply_en)
  dontTouch(Mcheck_call)
  dontTouch(Mchecke_call)
  dontTouch(Srecode_call)
  dontTouch(start_check)
  dontTouch(check_busy)
  dontTouch(score_apply)
  dontTouch(rf_ready)
  dontTouch(mcore_checking)
  dontTouch(score_check_done)
  dontTouch(score_comp_done)
  dontTouch(score_check_left)
  dontTouch(score_return_pc)
  dontTouch(past_sign)
  dontTouch(slave_is_umode)
  // dontTouch(trace_priv)
  // dontTouch(trace_priv_next)
  // dontTouch(trace_ppn)
  // dontTouch(trace_ppn_next)
  // dontTouch(priv_change)
  // dontTouch(ppn_change)
  // dontTouch(task_notchange)

  val custom_regbool = RegInit(false.B)
  val slave_rece_en = RegInit(false.B)
  
  val HartID = VecInit(GlobalParams.List_hartid.map(_.U))
  // val HartID1 = VecInit(GlobalParams.List_hartid1.map(_.U))
  // val HartID2 = VecInit(GlobalParams.List_hartid2.map(_.U))
  // dontTouch(HartID1)
  // dontTouch(HartID2)
  dontTouch(HartID)

  // val NumMaster = if(GlobalParams.List_hartid.contains(tileParams.hartId)) 
  //                   RegInit(GlobalParams.Num_Mastercores.U(4.W))
  //                 else 
  //                   RegInit(GlobalParams.Num_Mastercores2.U(4.W))
  // val NumSlave = if(GlobalParams.List_hartid1.contains(tileParams.hartId)) 
  //                   RegInit(GlobalParams.Num_Slavecores1.U(4.W))
  //                else 
  //                   RegInit(GlobalParams.Num_Slavecores2.U(4.W))
  // val MasterID = if(GlobalParams.List_hartid1.contains(tileParams.hartId)) 
  //                   RegInit(VecInit((GlobalParams.List_MasterId1 ++ List.fill(GlobalParams.Num_Groupcores - GlobalParams.Num_Mastercores1)(15)).map(_.U)))
  //                else
  //                   RegInit(VecInit((GlobalParams.List_MasterId2 ++ List.fill(GlobalParams.Num_Groupcores - GlobalParams.Num_Mastercores2)(15)).map(_.U)))
  // val SlaveID = if(GlobalParams.List_hartid1.contains(tileParams.hartId)) 
  //                   RegInit(VecInit((GlobalParams.List_SlaveId1 ++ List.fill(GlobalParams.Num_Groupcores - GlobalParams.Num_Slavecores1)(15)).map(_.U)))
  //                else
  //                   RegInit(VecInit((GlobalParams.List_SlaveId2 ++ List.fill(GlobalParams.Num_Groupcores - GlobalParams.Num_Slavecores2)(15)).map(_.U)))
  
  val NumMaster = RegInit(GlobalParams.Num_Mastercores.U(4.W))
  val NumSlave  = RegInit(GlobalParams.Num_Slavecores.U(4.W))
  val MasterID  = RegInit(VecInit((GlobalParams.List_MasterId ++ List.fill(GlobalParams.Num_Groupcores - GlobalParams.Num_Mastercores)(15)).map(_.U)))             
  val SlaveID   = RegInit(VecInit((GlobalParams.List_SlaveId ++ List.fill(GlobalParams.Num_Groupcores - GlobalParams.Num_Slavecores)(15)).map(_.U)))
                

  // val isGruop1           = HartID1.contains(io.hartid)
  // val isGruop2           = HartID2.contains(io.hartid)

  val isMaster           = MasterID.contains(io.hartid)
  val isSlave            = SlaveID.contains(io.hartid)
  val MFIFO_almostfull   = isMaster && FIFO.io.almostfull
  val SFIFO_almostempty  = isSlave && FIFO.io.almostempty
  val mulicheckcore_mode = (NumMaster < NumSlave)

  val score_return     = score_check_done && score_comp_done && isSlave && user_mode
  dontTouch(score_return)
  val mcore_free             = RegInit(VecInit(Seq.fill(GlobalParams.Num_Groupcores)(false.B)))//indicate mcore's scores are checking
  val mcore_check_free       = mcore_free.reduce(_ && _) && isMaster
  val score_check_overtaking = WireInit(false.B)
  dontTouch(mcore_free)
  dontTouch(mcore_check_free)

  //tw'sdefinition
  val instcoun = Module(new Instcounter())
  val isa = Module(new ISS())
  val copyvalid  = RegInit(false.B)
  val start_copyvalid  = RegInit(false.B)
  val rf_sentvalid  = RegInit(false.B)
  val q_copyvalid = Wire(Bool())  
  dontTouch(rf_sentvalid)
  dontTouch(start_copyvalid)
  
  val en_copyvalid = Wire(Bool())
  val pc_reg = RegInit(0.U(40.W))
  val intpc_reg = RegInit(0.U(40.W))
  val fsign = RegInit(true.B)
  dontTouch(fsign)
  
  val sbo = Wire(Vec(33,Bool()))
  val fp_sbo = Wire(Vec(33,Bool()))
  val statesignal = Wire(Bool())

  //ym's definition
  val ls_sentvalid = WireInit(false.B)
  val ls_data = WireInit(0.U(256.W))
  dontTouch(ls_sentvalid)
  dontTouch(ls_data)


  when(score_return){
    score_check_done := false.B
    score_comp_done  := false.B
    slave_rece_en    := false.B
    have_jumped      := false.B
    have_reve        := false.B
  }

  when(score_comp_done){
    slave_rece_en    := false.B
  }


  // dontTouch(isGruop1)
  // dontTouch(isGruop2)
  dontTouch(isMaster)
  dontTouch(isSlave)
  dontTouch(MFIFO_almostfull)
  dontTouch(SFIFO_almostempty)
  /*                  
  val NumMaster1 = RegInit(GlobalParams.Num_Mastercores1.U(4.W))
  val NumSlave1 = RegInit(GlobalParams.Num_Slavecores1.U(4.W))
  val MasterID1 = RegInit(VecInit((GlobalParams.List_MasterId1 ++ List.fill(GlobalParams.Num_Groupcores - GlobalParams.Num_Mastercores1)(15)).map(_.U)))
  val SlaveID1 = RegInit(VecInit((GlobalParams.List_SlaveId1 ++ List.fill(GlobalParams.Num_Groupcores - GlobalParams.Num_Slavecores1)(15)).map(_.U)))
  val NumMaster2 = RegInit(GlobalParams.Num_Mastercores2.U(4.W))
  val NumSlave2 = RegInit(GlobalParams.Num_Slavecores2.U(4.W))
  val MasterID2 = RegInit(VecInit((GlobalParams.List_MasterId2 ++ List.fill(GlobalParams.Num_Groupcores - GlobalParams.Num_Mastercores2)(15)).map(_.U)))
  val SlaveID2 = RegInit(VecInit((GlobalParams.List_MasterId2 ++ List.fill(GlobalParams.Num_Groupcores - GlobalParams.Num_Mastercores2)(15)).map(_.U)))
  */

  //Mux control signals and assertion
  // val reg_sels = if(GlobalParams.List_hartid1.contains(tileParams.hartId))
  //                   RegInit(VecInit(Seq(
  //                     VecInit(false.B, false.B, false.B, true.B),
  //                     VecInit(false.B, false.B, true.B, false.B),
  //                     VecInit(false.B, false.B, false.B, false.B),
  //                     VecInit(false.B, false.B, false.B, false.B)
  //                     )))
  //                 else
  //                   RegInit(VecInit(Seq(
  //                     VecInit(false.B, false.B, true.B, false.B),
  //                     VecInit(false.B, false.B, false.B, true.B),
  //                     VecInit(false.B, false.B, false.B, false.B),
  //                     VecInit(false.B, false.B, false.B, false.B)
  //                     )))

  val reg_sels = RegInit(VecInit(Seq(
                      VecInit(false.B, false.B, false.B, true.B),
                      VecInit(false.B, false.B, true.B, false.B),
                      VecInit(false.B, false.B, false.B, false.B),
                      VecInit(false.B, false.B, false.B, false.B)
                      )))
  val reg_slavesels = RegInit(VecInit((0 until 4).map {i =>
    Reverse(Cat(reg_sels.map(_(i))))
  }))

  // //assert for matching selection signals and ID
  // for(i <- 0 until GlobalParams.Num_Groupcores){
  //   when(isGruop1){
  //     when(i.U < NumSlave){
  //       assert(!(reg_sels(SlaveID(i)).reduce(_ || _)))
  //       assert(PopCount(reg_slavesels(SlaveID(i))) <= 1.U)
  //     }
  //     when(i.U < NumMaster){
  //       assert(PopCount(reg_slavesels(MasterID(i))) === 0.U)
  //     }
  //   }.otherwise{
  //     when(i.U < NumSlave){
  //       assert(!(reg_sels(SlaveID(i) - 4.U).reduce(_ || _)))
  //       assert(PopCount(reg_slavesels(SlaveID(i) - 4.U)) <= 1.U)
  //     }
      
      
  //     when(i.U < NumMaster){
  //       assert(PopCount(reg_slavesels(MasterID(i) - 4.U)) === 0.U)
  //     }
  //   }
  // }
  

  //debug singals
  dontTouch(custom_regbool)
  dontTouch(slave_rece_en)
  dontTouch(reg_sels)
  dontTouch(reg_slavesels)
  dontTouch(custom_reg)
  dontTouch(custom_regin)
  dontTouch(io.custom_FIFOin)
  dontTouch(io.custom_FIFOout)
  //dontTouch(counter)
  dontTouch(NumMaster)
  dontTouch(NumSlave)
  dontTouch(MasterID)
  dontTouch(SlaveID)
  // dontTouch(io.numMasterin)
  // dontTouch(io.numMasterout)
  // dontTouch(io.numSlavein)
  // dontTouch(io.numSlaveout)
  // dontTouch(io.MasterIDin)
  // dontTouch(io.MasterIDout)
  // dontTouch(io.SlaveIDin)
  // dontTouch(io.SlaveIDout)
  dontTouch(have_jumped)
  dontTouch(ls_sign)
  dontTouch(ls_inst)
  dontTouch(ls_addr)
  dontTouch(ld_value)
  dontTouch(st_value)
  dontTouch(mode_signs)
  dontTouch(mode_signe)
  dontTouch(rf_count)
  dontTouch(rf_wrap)
  //counter := counter + 1.U

//custom define end


  val clock_en_reg = RegInit(true.B)
  val long_latency_stall = Reg(Bool())
  val id_reg_pause = Reg(Bool())
  val imem_might_request_reg = Reg(Bool())
  val clock_en = WireDefault(true.B)
  val gated_clock =
    if (!rocketParams.clockGate) clock
    else ClockGate(clock, clock_en, "rocket_clock_gate")

  class RocketImpl { // entering gated-clock domain

  // performance counters
  def pipelineIDToWB[T <: Data](x: T): T =
    RegEnable(RegEnable(RegEnable(x, !ctrl_killd), ex_pc_valid), mem_pc_valid)
  val perfEvents = new EventSets(Seq(
    new EventSet((mask, hits) => Mux(wb_xcpt, mask(0), wb_valid && pipelineIDToWB((mask & hits).orR)), Seq(
      ("exception", () => false.B),
      ("load", () => id_ctrl.mem && id_ctrl.mem_cmd === M_XRD && !id_ctrl.fp),
      ("store", () => id_ctrl.mem && id_ctrl.mem_cmd === M_XWR && !id_ctrl.fp),
      ("amo", () => usingAtomics.B && id_ctrl.mem && (isAMO(id_ctrl.mem_cmd) || id_ctrl.mem_cmd.isOneOf(M_XLR, M_XSC))),
      ("system", () => id_ctrl.csr =/= CSR.N),
      ("arith", () => id_ctrl.wxd && !(id_ctrl.jal || id_ctrl.jalr || id_ctrl.mem || id_ctrl.fp || id_ctrl.mul || id_ctrl.div || id_ctrl.csr =/= CSR.N)),
      ("branch", () => id_ctrl.branch),
      ("jal", () => id_ctrl.jal),
      ("jalr", () => id_ctrl.jalr))
      ++ (if (!usingMulDiv) Seq() else Seq(
        ("mul", () => if (pipelinedMul) id_ctrl.mul else id_ctrl.div && (id_ctrl.alu_fn & aluFn.FN_DIV) =/= aluFn.FN_DIV),
        ("div", () => if (pipelinedMul) id_ctrl.div else id_ctrl.div && (id_ctrl.alu_fn & aluFn.FN_DIV) === aluFn.FN_DIV)))
      ++ (if (!usingFPU) Seq() else Seq(
        ("fp load", () => id_ctrl.fp && io.fpu.dec.ldst && io.fpu.dec.wen),
        ("fp store", () => id_ctrl.fp && io.fpu.dec.ldst && !io.fpu.dec.wen),
        ("fp add", () => id_ctrl.fp && io.fpu.dec.fma && io.fpu.dec.swap23),
        ("fp mul", () => id_ctrl.fp && io.fpu.dec.fma && !io.fpu.dec.swap23 && !io.fpu.dec.ren3),
        ("fp mul-add", () => id_ctrl.fp && io.fpu.dec.fma && io.fpu.dec.ren3),
        ("fp div/sqrt", () => id_ctrl.fp && (io.fpu.dec.div || io.fpu.dec.sqrt)),
        ("fp other", () => id_ctrl.fp && !(io.fpu.dec.ldst || io.fpu.dec.fma || io.fpu.dec.div || io.fpu.dec.sqrt))))),
    new EventSet((mask, hits) => (mask & hits).orR, Seq(
      ("load-use interlock", () => id_ex_hazard && ex_ctrl.mem || id_mem_hazard && mem_ctrl.mem || id_wb_hazard && wb_ctrl.mem),
      ("long-latency interlock", () => id_sboard_hazard),
      ("csr interlock", () => id_ex_hazard && ex_ctrl.csr =/= CSR.N || id_mem_hazard && mem_ctrl.csr =/= CSR.N || id_wb_hazard && wb_ctrl.csr =/= CSR.N),
      ("I$ blocked", () => icache_blocked),
      ("D$ blocked", () => id_ctrl.mem && dcache_blocked),
      ("branch misprediction", () => take_pc_mem && mem_direction_misprediction),
      ("control-flow target misprediction", () => take_pc_mem && mem_misprediction && mem_cfi && !mem_direction_misprediction && !icache_blocked),
      ("flush", () => wb_reg_flush_pipe),
      ("replay", () => replay_wb))
      ++ (if (!usingMulDiv) Seq() else Seq(
        ("mul/div interlock", () => id_ex_hazard && (ex_ctrl.mul || ex_ctrl.div) || id_mem_hazard && (mem_ctrl.mul || mem_ctrl.div) || id_wb_hazard && wb_ctrl.div)))
      ++ (if (!usingFPU) Seq() else Seq(
        ("fp interlock", () => id_ex_hazard && ex_ctrl.fp || id_mem_hazard && mem_ctrl.fp || id_wb_hazard && wb_ctrl.fp || id_ctrl.fp && id_stall_fpu)))),
    new EventSet((mask, hits) => (mask & hits).orR, Seq(
      ("I$ miss", () => io.imem.perf.acquire),
      ("D$ miss", () => io.dmem.perf.acquire),
      ("D$ release", () => io.dmem.perf.release),
      ("ITLB miss", () => io.imem.perf.tlbMiss),
      ("DTLB miss", () => io.dmem.perf.tlbMiss),
      ("L2 TLB miss", () => io.ptw.perf.l2miss)))))

  val pipelinedMul = usingMulDiv && mulDivParams.mulUnroll == xLen
  val decode_table = {
    require(!usingRoCC || !rocketParams.useSCIE)
    (if (usingMulDiv) new MDecode(pipelinedMul, aluFn) +: (xLen > 32).option(new M64Decode(pipelinedMul, aluFn)).toSeq else Nil) ++:
    (if (usingAtomics) new ADecode(aluFn) +: (xLen > 32).option(new A64Decode(aluFn)).toSeq else Nil) ++:
    (if (fLen >= 32)    new FDecode(aluFn) +: (xLen > 32).option(new F64Decode(aluFn)).toSeq else Nil) ++:
    (if (fLen >= 64)    new DDecode(aluFn) +: (xLen > 32).option(new D64Decode(aluFn)).toSeq else Nil) ++:
    (if (minFLen == 16) new HDecode(aluFn) +: (xLen > 32).option(new H64Decode(aluFn)).toSeq ++: (fLen >= 64).option(new HDDecode(aluFn)).toSeq else Nil) ++:
    (usingRoCC.option(new RoCCDecode(aluFn))) ++:
    (rocketParams.useSCIE.option(new SCIEDecode(aluFn))) ++:
    (if (usingBitManip) new ZBADecode +: (xLen == 64).option(new ZBA64Decode).toSeq ++: new ZBBMDecode +: new ZBBORCBDecode +: new ZBCRDecode +: new ZBSDecode +: (xLen == 32).option(new ZBS32Decode).toSeq ++: (xLen == 64).option(new ZBS64Decode).toSeq ++: new ZBBSEDecode +: new ZBBCDecode +: (xLen == 64).option(new ZBBC64Decode).toSeq else Nil) ++:
    (if (usingBitManip && !usingBitManipCrypto) (xLen == 32).option(new ZBBZE32Decode).toSeq ++: (xLen == 64).option(new ZBBZE64Decode).toSeq else Nil) ++:
    (if (usingBitManip || usingBitManipCrypto) new ZBBNDecode +: new ZBCDecode +: new ZBBRDecode +: (xLen == 32).option(new ZBBR32Decode).toSeq ++: (xLen == 64).option(new ZBBR64Decode).toSeq ++: (xLen == 32).option(new ZBBREV832Decode).toSeq ++: (xLen == 64).option(new ZBBREV864Decode).toSeq else Nil) ++:
    (if (usingBitManipCrypto) new ZBKXDecode +: new ZBKBDecode +: (xLen == 32).option(new ZBKB32Decode).toSeq ++: (xLen == 64).option(new ZBKB64Decode).toSeq else Nil) ++:
    (if (usingCryptoNIST) (xLen == 32).option(new ZKND32Decode).toSeq ++: (xLen == 64).option(new ZKND64Decode).toSeq else Nil) ++:
    (if (usingCryptoNIST) (xLen == 32).option(new ZKNE32Decode).toSeq ++: (xLen == 64).option(new ZKNE64Decode).toSeq else Nil) ++:
    (if (usingCryptoNIST) new ZKNHDecode +: (xLen == 32).option(new ZKNH32Decode).toSeq ++: (xLen == 64).option(new ZKNH64Decode).toSeq else Nil) ++:
    (usingCryptoSM.option(new ZKSDecode)) ++:
    (if (xLen == 32) new I32Decode(aluFn) else new I64Decode(aluFn)) +:
    (usingVM.option(new SVMDecode(aluFn))) ++:
    (usingSupervisor.option(new SDecode(aluFn))) ++:
    (usingHypervisor.option(new HypervisorDecode(aluFn))) ++:
    ((usingHypervisor && (xLen == 64)).option(new Hypervisor64Decode(aluFn))) ++:
    (usingDebug.option(new DebugDecode(aluFn))) ++:
    (usingNMI.option(new NMIDecode(aluFn))) ++:
    (usingConditionalZero.option(new ConditionalZeroDecode(aluFn))) ++:
    Seq(new FenceIDecode(tile.dcache.flushOnFenceI, aluFn)) ++:
    coreParams.haveCFlush.option(new CFlushDecode(tile.dcache.canSupportCFlushLine, aluFn)) ++:
    rocketParams.haveCease.option(new CeaseDecode(aluFn)) ++:
    Seq(new IDecode(aluFn))
  } flatMap(_.table)

  val ex_ctrl     = Reg(new IntCtrlSigs(aluFn))
  val mem_ctrl    = Reg(new IntCtrlSigs(aluFn))
  val wb_ctrl     = Reg(new IntCtrlSigs(aluFn))
  //val comit_ctrl  = Reg(new IntCtrlSigs(aluFn))

  val ex_reg_xcpt_interrupt  = Reg(Bool())
  val ex_reg_valid           = Reg(Bool())
  val ex_reg_rvc             = Reg(Bool())
  val ex_reg_btb_resp        = Reg(new BTBResp)
  val ex_reg_xcpt            = Reg(Bool())
  val ex_reg_flush_pipe      = Reg(Bool())
  val ex_reg_load_use        = Reg(Bool())
  val ex_reg_cause           = Reg(UInt())
  val ex_reg_replay = Reg(Bool())
  val ex_reg_pc = Reg(UInt())
  val ex_reg_mem_size = Reg(UInt())
  val ex_reg_hls = Reg(Bool())
  val ex_reg_inst = Reg(Bits())
  val ex_reg_raw_inst = Reg(UInt())
  val ex_scie_unpipelined = Reg(Bool())
  val ex_scie_pipelined = Reg(Bool())
  val ex_reg_wphit            = Reg(Vec(nBreakpoints, Bool()))

  val mem_reg_xcpt_interrupt  = Reg(Bool())
  val mem_reg_valid           = Reg(Bool())
  val mem_reg_rvc             = Reg(Bool())
  val mem_reg_btb_resp        = Reg(new BTBResp)
  val mem_reg_xcpt            = Reg(Bool())
  val mem_reg_replay          = Reg(Bool())
  val mem_reg_flush_pipe      = Reg(Bool())
  val mem_reg_cause           = Reg(UInt())
  val mem_reg_slow_bypass     = Reg(Bool())
  val mem_reg_load            = Reg(Bool())
  val mem_reg_store           = Reg(Bool())
  val mem_reg_sfence = Reg(Bool())
  val mem_reg_pc = Reg(UInt())
  val mem_reg_inst = Reg(Bits())
  val mem_reg_mem_size = Reg(UInt())
  val mem_reg_hls_or_dv = Reg(Bool())
  val mem_reg_raw_inst = Reg(UInt())
  val mem_scie_unpipelined = Reg(Bool())
  val mem_scie_pipelined = Reg(Bool())
  val mem_reg_wdata = Reg(Bits())
  val mem_reg_rs2 = Reg(Bits())
  val mem_br_taken = Reg(Bool())
  val take_pc_mem = Wire(Bool())
  val mem_reg_wphit          = Reg(Vec(nBreakpoints, Bool()))

  val wb_reg_valid           = Reg(Bool())
  val wb_reg_xcpt            = Reg(Bool())
  val wb_reg_replay          = Reg(Bool())
  val wb_reg_flush_pipe      = Reg(Bool())
  val wb_reg_cause           = Reg(UInt())
  val wb_reg_sfence = Reg(Bool())
  val wb_reg_pc = Reg(UInt())
  val wb_reg_mem_size = Reg(UInt())
  val wb_reg_hls_or_dv = Reg(Bool())
  val wb_reg_hfence_v = Reg(Bool())
  val wb_reg_hfence_g = Reg(Bool())
  val wb_reg_inst = Reg(Bits())
  val wb_reg_raw_inst = Reg(UInt())
  val wb_reg_wdata = Reg(Bits())
  val wb_reg_rs2 = Reg(Bits())
  val take_pc_wb = Wire(Bool())
  val wb_reg_wphit           = Reg(Vec(nBreakpoints, Bool()))

  val take_pc_mem_wb = take_pc_wb || take_pc_mem
  val take_pc = take_pc_mem_wb

  //trace each stage inst
  val ex_trace_inst  = (if(usingCompressed) Cat(Mux(ex_reg_raw_inst(1, 0).andR, ex_reg_inst >> 16, 0.U), ex_reg_raw_inst(15, 0)) else ex_reg_inst)
  val mem_trace_inst = (if(usingCompressed) Cat(Mux(mem_reg_raw_inst(1, 0).andR, mem_reg_inst >> 16, 0.U), mem_reg_raw_inst(15, 0)) else mem_reg_inst)
  val wb_trace_inst  = (if(usingCompressed) Cat(Mux(wb_reg_raw_inst(1, 0).andR, wb_reg_inst >> 16, 0.U), wb_reg_raw_inst(15, 0)) else wb_reg_inst)
  // check custom wires
  val fifo_ready = !FIFO.io.empty && FIFO.io.out.bits(255, 224) === "h_dead_beef".U //&& FIFO.io.out.bits(223, 192) === ex_reg_inst
  //val check_req_ex_ready = fifo_ready && isSlave && FIFO.io.out.bits(223, 192) === (if(usingCompressed) Cat(Mux(ex_reg_raw_inst(1, 0).andR, ex_reg_inst >> 16, 0.U), ex_reg_raw_inst(15, 0)) else ex_reg_inst)
  val check_req_ready    = fifo_ready && isSlave && FIFO.io.out.bits(223, 192) === (if(usingCompressed) Cat(Mux(mem_reg_raw_inst(1, 0).andR, mem_reg_inst >> 16, 0.U), mem_reg_raw_inst(15, 0)) else mem_reg_inst)
  val check_req_data_ld  = WireInit(0.U(64.W))
  val check_req_data_st  = WireInit(0.U(64.W))
  val check_req_addr     = WireInit(0.U(64.W))
  // req is ready when fifo can provide data, otherwise replay_ex is triggered 
  // resp is bound to be valid, since there is no "cache-miss" from fifo
  // fifo_raw [255:0]
  //val fifo_data = WireInit(0.U(64.W))

  //val dcache = new DCache(tile.staticIdForMetadataUseOnly, tile.crossing)(p)
  val check_resp_valid_test    = WireInit(false.B)
  val check_resp_replay   = WireInit(false.B)
  val check_replay_ready   = WireInit(false.B)
  val check_replay_valid   = WireInit(false.B)

  val check_resp_valid    = WireInit(false.B)
  val check_s2_nack       = WireInit(false.B)

  val check_replay_flag   = RegInit(false.B)
  val check_replay_inst   = RegInit(0.U(32.W))
  val check_resp_data_ld  = RegInit(0.U(64.W))
  val check_resp_data_st  = RegInit(0.U(64.W))
  val check_resp_addr     = RegInit(0.U(64.W))
  val check_resp_waddr    = WireInit(0.U(5.W))
  val check_resp_valid_same = check_resp_valid === check_resp_valid_test
  

  val check_addr       = RegInit(true.B)
  val check_data       = RegInit(true.B)

  val check_cp         = RegInit(true.B)
  val check_rfcp       = RegInit(true.B)
  val check_fprfcp     = RegInit(true.B)
  check_cp      := check_rfcp && check_fprfcp
  

  when(check_req_ready){
    check_resp_data_ld    := check_req_data_ld
    check_resp_data_st := check_req_data_st
    check_resp_addr    := check_req_addr
  }
  //check_resp_data := fifo_data  
  // original dmem req after two cycles
  val check_resp_xpu   = !wb_ctrl.fp
  val check_resp_fpu   = wb_ctrl.fp
  val check_resp_size  = wb_reg_mem_size

  //dontTouch(fifo_data)
  dontTouch(fifo_ready)
  dontTouch(check_req_ready)
  dontTouch(check_req_data_ld)
  dontTouch(check_req_data_st)
  dontTouch(check_req_addr)
  dontTouch(check_resp_data_ld)
  dontTouch(check_resp_data_st)
  dontTouch(check_resp_xpu)
  dontTouch(check_resp_fpu)
  dontTouch(check_resp_size)
  dontTouch(check_resp_valid)
  dontTouch(check_addr)
  dontTouch(check_data)
  dontTouch(check_cp)
  dontTouch(check_rfcp)
  dontTouch(check_fprfcp)
  dontTouch(rf_sign)
  dontTouch(pc_sign)
  dontTouch(rf_datatype)
  dontTouch(widx)
  dontTouch(wrf_data)
  dontTouch(wfprf_data)
  dontTouch(check_resp_valid_test)
  dontTouch(check_resp_valid_same)

  // test_fifo.io.enq.bits    := custom_reg
  // test_fifo.io.enq.valid   := ex_reg_valid
  // test_fifo.io.deq.ready   := test_fifo1.io.enq.ready

  // test_fifo1.io.enq.bits   := test_fifo.io.deq.bits
  // test_fifo1.io.enq.valid  := test_fifo.io.deq.valid
  // test_fifo1.io.deq.ready   := wb_reg_valid

  // dontTouch(test_fifo.io.enq.ready)
  // dontTouch(test_fifo.io.deq.bits)
  // dontTouch(test_fifo.io.deq.valid)
  // dontTouch(test_fifo1.io.enq.ready)
  // dontTouch(test_fifo1.io.deq.bits)
  // dontTouch(test_fifo1.io.deq.valid)
  // decode stage
  val ibuf = Module(new IBuf)
  val id_expanded_inst = ibuf.io.inst.map(_.bits.inst)
  val id_raw_inst = ibuf.io.inst.map(_.bits.raw)
  val id_inst = id_expanded_inst.map(_.bits)
  ibuf.io.imem <> io.imem.resp
  ibuf.io.kill := take_pc
  dontTouch(id_inst(0))
  val id_rocc_inst = id_inst(0).asTypeOf(new RoCCInstruction())

  require(decodeWidth == 1 /* TODO */ && retireWidth == decodeWidth)
  require(!(coreParams.useRVE && coreParams.fpu.nonEmpty), "Can't select both RVE and floating-point")
  require(!(coreParams.useRVE && coreParams.useHypervisor), "Can't select both RVE and Hypervisor")
  val id_ctrl = Wire(new IntCtrlSigs(aluFn)).decode(id_inst(0), decode_table)
  val lgNXRegs = if (coreParams.useRVE) 4 else 5
  val regAddrMask = (1 << lgNXRegs) - 1

  def decodeReg(x: UInt) = (x.extract(x.getWidth-1, lgNXRegs).asBool, x(lgNXRegs-1, 0))
  val (id_raddr3_illegal, id_raddr3) = decodeReg(id_expanded_inst(0).rs3)
  val (id_raddr2_illegal, id_raddr2) = decodeReg(id_expanded_inst(0).rs2)
  val (id_raddr1_illegal, id_raddr1) = decodeReg(id_expanded_inst(0).rs1)
  val (id_waddr_illegal,  id_waddr)  = decodeReg(id_expanded_inst(0).rd)

  val id_load_use = Wire(Bool())
  val id_reg_fence = RegInit(false.B)
  val id_ren = IndexedSeq(id_ctrl.rxs1, id_ctrl.rxs2)
  val id_raddr = IndexedSeq(id_raddr1, id_raddr2)
  val rf = new RegFile(regAddrMask, xLen)
  val id_rs = id_raddr.map(rf.read _)
  val ctrl_killd = Wire(Bool())
  val id_npc = (ibuf.io.pc.asSInt + ImmGen(IMM_UJ, id_inst(0))).asUInt

  val csr = Module(new CSRFile(perfEvents, coreParams.customCSRs.decls, tile.roccCSRs.flatten))
  val id_csr_en = id_ctrl.csr.isOneOf(CSR.S, CSR.C, CSR.W)
  val id_system_insn = id_ctrl.csr === CSR.I
  val id_csr_ren = id_ctrl.csr.isOneOf(CSR.S, CSR.C) && id_expanded_inst(0).rs1 === 0.U
  val id_csr = Mux(id_system_insn && id_ctrl.mem, CSR.N, Mux(id_csr_ren, CSR.R, id_ctrl.csr))
  val id_csr_flush = id_system_insn || (id_csr_en && !id_csr_ren && csr.io.decode(0).write_flush)

  val id_scie_decoder = if (!rocketParams.useSCIE) WireDefault(0.U.asTypeOf(new SCIEDecoderInterface)) else {
    val d = Module(new SCIEDecoder)
    assert(!io.imem.resp.valid || PopCount(d.io.unpipelined :: d.io.pipelined :: d.io.multicycle :: Nil) <= 1.U)
    d.io.insn := id_raw_inst(0)
    d.io
  }
  val id_illegal_rnum = if (usingCryptoNIST) (id_ctrl.zkn && aluFn.isKs1(id_ctrl.alu_fn) && id_inst(0)(23,20) > 0xA.U(4.W)) else false.B
  val id_illegal_insn = !id_ctrl.legal ||
    (id_ctrl.mul || id_ctrl.div) && !csr.io.status.isa('m'-'a') ||
    id_ctrl.amo && !csr.io.status.isa('a'-'a') ||
    id_ctrl.fp && (csr.io.decode(0).fp_illegal || io.fpu.illegal_rm) ||
    id_ctrl.dp && !csr.io.status.isa('d'-'a') ||
    ibuf.io.inst(0).bits.rvc && !csr.io.status.isa('c'-'a') ||
    id_raddr2_illegal && !id_ctrl.scie && id_ctrl.rxs2 ||
    id_raddr1_illegal && !id_ctrl.scie && id_ctrl.rxs1 ||
    id_waddr_illegal && !id_ctrl.scie && id_ctrl.wxd ||
    id_ctrl.rocc && csr.io.decode(0).rocc_illegal ||
    id_ctrl.scie && !(id_scie_decoder.unpipelined || id_scie_decoder.pipelined) ||
    id_csr_en && (csr.io.decode(0).read_illegal || !id_csr_ren && csr.io.decode(0).write_illegal) ||
    !ibuf.io.inst(0).bits.rvc && (id_system_insn && csr.io.decode(0).system_illegal) ||
    id_illegal_rnum
  val id_virtual_insn = id_ctrl.legal &&
    ((id_csr_en && !(!id_csr_ren && csr.io.decode(0).write_illegal) && csr.io.decode(0).virtual_access_illegal) ||
     (!ibuf.io.inst(0).bits.rvc && id_system_insn && csr.io.decode(0).virtual_system_illegal))
  // stall decode for fences (now, for AMO.rl; later, for AMO.aq and FENCE)
  val id_amo_aq = id_inst(0)(26)
  val id_amo_rl = id_inst(0)(25)
  val id_fence_pred = id_inst(0)(27,24)
  val id_fence_succ = id_inst(0)(23,20)
  val id_fence_next = id_ctrl.fence || id_ctrl.amo && id_amo_aq
  // Modified
  val id_mem_busy = Mux(check_busy && user_mode, false.B, !io.dmem.ordered || io.dmem.req.valid)
  when (!id_mem_busy) { id_reg_fence := false.B }
  val id_rocc_busy = usingRoCC.B &&
    (io.rocc.busy || ex_reg_valid && ex_ctrl.rocc ||
     mem_reg_valid && mem_ctrl.rocc || wb_reg_valid && wb_ctrl.rocc)
  val id_do_fence = WireDefault(id_rocc_busy && id_ctrl.fence ||
    id_mem_busy && (id_ctrl.amo && id_amo_rl || id_ctrl.fence_i || id_reg_fence && (id_ctrl.mem || id_ctrl.rocc)))

  val bpu = Module(new BreakpointUnit(nBreakpoints))
  bpu.io.status := csr.io.status
  bpu.io.bp := csr.io.bp
  bpu.io.pc := ibuf.io.pc
  bpu.io.ea := mem_reg_wdata
  bpu.io.mcontext := csr.io.mcontext
  bpu.io.scontext := csr.io.scontext

  val id_xcpt0 = ibuf.io.inst(0).bits.xcpt0
  val id_xcpt1 = ibuf.io.inst(0).bits.xcpt1
  val (id_xcpt, id_cause) = checkExceptions(List(
    (csr.io.interrupt, csr.io.interrupt_cause),
    (bpu.io.debug_if,  CSR.debugTriggerCause.U),
    (bpu.io.xcpt_if,   Causes.breakpoint.U),
    (id_xcpt0.pf.inst, Causes.fetch_page_fault.U),
    (id_xcpt0.gf.inst, Causes.fetch_guest_page_fault.U),
    (id_xcpt0.ae.inst, Causes.fetch_access.U),
    (id_xcpt1.pf.inst, Causes.fetch_page_fault.U),
    (id_xcpt1.gf.inst, Causes.fetch_guest_page_fault.U),
    (id_xcpt1.ae.inst, Causes.fetch_access.U),
    (id_virtual_insn,  Causes.virtual_instruction.U),
    (id_illegal_insn,  Causes.illegal_instruction.U)))

  val idCoverCauses = List(
    (CSR.debugTriggerCause, "DEBUG_TRIGGER"),
    (Causes.breakpoint, "BREAKPOINT"),
    (Causes.fetch_access, "FETCH_ACCESS"),
    (Causes.illegal_instruction, "ILLEGAL_INSTRUCTION")
  ) ++ (if (usingVM) List(
    (Causes.fetch_page_fault, "FETCH_PAGE_FAULT")
  ) else Nil)
  coverExceptions(id_xcpt, id_cause, "DECODE", idCoverCauses)

  val dcache_bypass_data =
    if (fastLoadByte) Mux(check_busy && user_mode, check_resp_data_ld, io.dmem.resp.bits.data(xLen-1, 0))
    else if (fastLoadWord) Mux(check_busy && user_mode, check_resp_data_ld, io.dmem.resp.bits.data_word_bypass(xLen-1, 0))
    else wb_reg_wdata
  dontTouch(dcache_bypass_data)

  // detect bypass opportunities
  val ex_waddr = ex_reg_inst(11,7) & regAddrMask.U
  val mem_waddr = mem_reg_inst(11,7) & regAddrMask.U
  val wb_waddr = wb_reg_inst(11,7) & regAddrMask.U
  val bypass_sources = IndexedSeq(
    (true.B, 0.U, 0.U), // treat reading x0 as a bypass
    (ex_reg_valid && ex_ctrl.wxd, ex_waddr, mem_reg_wdata),
    (mem_reg_valid && mem_ctrl.wxd && !mem_ctrl.mem, mem_waddr, wb_reg_wdata),
    (mem_reg_valid && mem_ctrl.wxd, mem_waddr, dcache_bypass_data))
  val id_bypass_src = id_raddr.map(raddr => bypass_sources.map(s => s._1 && s._2 === raddr))

  // execute stage
  val bypass_mux = bypass_sources.map(_._3)
  val ex_reg_rs_bypass = Reg(Vec(id_raddr.size, Bool()))
  val ex_reg_rs_lsb = Reg(Vec(id_raddr.size, UInt(log2Ceil(bypass_sources.size).W)))
  val ex_reg_rs_msb = Reg(Vec(id_raddr.size, UInt()))
  val ex_rs = for (i <- 0 until id_raddr.size)
    yield Mux(ex_reg_rs_bypass(i), bypass_mux(ex_reg_rs_lsb(i)), Cat(ex_reg_rs_msb(i), ex_reg_rs_lsb(i)))
  val ex_imm = ImmGen(ex_ctrl.sel_imm, ex_reg_inst)
  val ex_op1 = MuxLookup(ex_ctrl.sel_alu1, 0.S, Seq(
    A1_RS1 -> ex_rs(0).asSInt,
    A1_PC -> ex_reg_pc.asSInt))
  val ex_op2 = MuxLookup(ex_ctrl.sel_alu2, 0.S, Seq(
    A2_RS2 -> ex_rs(1).asSInt,
    A2_IMM -> ex_imm,
    A2_SIZE -> Mux(ex_reg_rvc, 2.S, 4.S)))

  val alu = Module(aluFn match {
    case _: ABLUFN => new ABLU
    case _: ALUFN => new ALU
  })
  alu.io.dw := ex_ctrl.alu_dw
  alu.io.fn := ex_ctrl.alu_fn
  alu.io.in2 := ex_op2.asUInt
  alu.io.in1 := ex_op1.asUInt

  val ex_scie_unpipelined_wdata = if (!rocketParams.useSCIE) 0.U else {
    val u = Module(new SCIEUnpipelined(xLen))
    u.io.insn := ex_reg_inst
    u.io.rs1 := ex_rs(0)
    u.io.rs2 := ex_rs(1)
    u.io.rd
  }

  val mem_scie_pipelined_wdata = if (!rocketParams.useSCIE) 0.U else {
    val u = Module(new SCIEPipelined(xLen))
    u.io.clock := Module.clock
    u.io.valid := ex_reg_valid && ex_scie_pipelined
    u.io.insn := ex_reg_inst
    u.io.rs1 := ex_rs(0)
    u.io.rs2 := ex_rs(1)
    u.io.rd
  }

  val ex_zbk_wdata = if (!usingBitManipCrypto && !usingBitManip) 0.U else {
    val zbk = Module(new BitManipCrypto(xLen))
    zbk.io.fn  := ex_ctrl.alu_fn
    zbk.io.dw  := ex_ctrl.alu_dw
    zbk.io.rs1 := ex_op1.asUInt
    zbk.io.rs2 := ex_op2.asUInt
    zbk.io.rd
  }

  val ex_zkn_wdata = if (!usingCryptoNIST) 0.U else {
    val zkn = Module(new CryptoNIST(xLen))
    zkn.io.fn   := ex_ctrl.alu_fn
    zkn.io.hl   := ex_reg_inst(27)
    zkn.io.bs   := ex_reg_inst(31,30)
    zkn.io.rs1  := ex_op1.asUInt
    zkn.io.rs2  := ex_op2.asUInt
    zkn.io.rd
  }

  val ex_zks_wdata = if (!usingCryptoSM) 0.U else {
    val zks = Module(new CryptoSM(xLen))
    zks.io.fn  := ex_ctrl.alu_fn
    zks.io.bs  := ex_reg_inst(31,30)
    zks.io.rs1 := ex_op1.asUInt
    zks.io.rs2 := ex_op2.asUInt
    zks.io.rd
  }

  // multiplier and divider
  val div = Module(new MulDiv(if (pipelinedMul) mulDivParams.copy(mulUnroll = 0) else mulDivParams, width = xLen, aluFn = aluFn))
  div.io.req.valid := ex_reg_valid && ex_ctrl.div
  div.io.req.bits.dw := ex_ctrl.alu_dw
  div.io.req.bits.fn := ex_ctrl.alu_fn
  div.io.req.bits.in1 := ex_rs(0)
  div.io.req.bits.in2 := ex_rs(1)
  div.io.req.bits.tag := ex_waddr
  val mul = pipelinedMul.option {
    val m = Module(new PipelinedMultiplier(xLen, 2, aluFn = aluFn))
    m.io.req.valid := ex_reg_valid && ex_ctrl.mul
    m.io.req.bits := div.io.req.bits
    m
  }

  ex_reg_valid := !ctrl_killd
  ex_reg_replay := !take_pc && ibuf.io.inst(0).valid && ibuf.io.inst(0).bits.replay
  ex_reg_xcpt := !ctrl_killd && id_xcpt
  ex_reg_xcpt_interrupt := !take_pc && ibuf.io.inst(0).valid && csr.io.interrupt

  when (!ctrl_killd) {
    ex_ctrl := id_ctrl
    ex_reg_rvc := ibuf.io.inst(0).bits.rvc
    ex_ctrl.csr := id_csr
    ex_scie_unpipelined := id_ctrl.scie && id_scie_decoder.unpipelined
    ex_scie_pipelined := id_ctrl.scie && id_scie_decoder.pipelined
    when (id_ctrl.fence && id_fence_succ === 0.U) { id_reg_pause := true.B }
    when (id_fence_next) { id_reg_fence := true.B }
    when (id_xcpt) { // pass PC down ALU writeback pipeline for badaddr
      ex_ctrl.alu_fn := aluFn.FN_ADD
      ex_ctrl.alu_dw := DW_XPR
      ex_ctrl.sel_alu1 := A1_RS1 // badaddr := instruction
      ex_ctrl.sel_alu2 := A2_ZERO
      when (id_xcpt1.asUInt.orR) { // badaddr := PC+2
        ex_ctrl.sel_alu1 := A1_PC
        ex_ctrl.sel_alu2 := A2_SIZE
        ex_reg_rvc := true.B
      }
      when (bpu.io.xcpt_if || id_xcpt0.asUInt.orR) { // badaddr := PC
        ex_ctrl.sel_alu1 := A1_PC
        ex_ctrl.sel_alu2 := A2_ZERO
      }
    }
    ex_reg_flush_pipe := id_ctrl.fence_i || id_csr_flush
                         
    ex_reg_load_use := id_load_use
    ex_reg_hls := usingHypervisor.B && id_system_insn && id_ctrl.mem_cmd.isOneOf(M_XRD, M_XWR, M_HLVX)
    ex_reg_mem_size := Mux(usingHypervisor.B && id_system_insn, id_inst(0)(27, 26), id_inst(0)(13, 12))
    when (id_ctrl.mem_cmd.isOneOf(M_SFENCE, M_HFENCEV, M_HFENCEG, M_FLUSH_ALL)) {
      ex_reg_mem_size := Cat(id_raddr2 =/= 0.U, id_raddr1 =/= 0.U)
    }
    when (id_ctrl.mem_cmd === M_SFENCE && csr.io.status.v) {
      ex_ctrl.mem_cmd := M_HFENCEV
    }
    if (tile.dcache.flushOnFenceI) {
      when (id_ctrl.fence_i) {
        ex_reg_mem_size := 0.U
      }
    }

    for (i <- 0 until id_raddr.size) {
      val do_bypass = id_bypass_src(i).reduce(_||_)
      val bypass_src = PriorityEncoder(id_bypass_src(i))
      ex_reg_rs_bypass(i) := do_bypass
      ex_reg_rs_lsb(i) := bypass_src
      when (id_ren(i) && !do_bypass) {
        ex_reg_rs_lsb(i) := id_rs(i)(log2Ceil(bypass_sources.size)-1, 0)
        ex_reg_rs_msb(i) := id_rs(i) >> log2Ceil(bypass_sources.size)
      }
    }
    when (id_illegal_insn || id_virtual_insn) {
      val inst = Mux(ibuf.io.inst(0).bits.rvc, id_raw_inst(0)(15, 0), id_raw_inst(0))
      ex_reg_rs_bypass(0) := false.B
      ex_reg_rs_lsb(0) := inst(log2Ceil(bypass_sources.size)-1, 0)
      ex_reg_rs_msb(0) := inst >> log2Ceil(bypass_sources.size)
    }
  }

  when (!ctrl_killd || csr.io.interrupt || ibuf.io.inst(0).bits.replay) {
    ex_reg_cause := id_cause
    ex_reg_inst := id_inst(0)
    ex_reg_raw_inst := id_raw_inst(0)
    ex_reg_pc := ibuf.io.pc
    ex_reg_btb_resp := ibuf.io.btb_resp
    ex_reg_wphit := bpu.io.bpwatch.map { bpw => bpw.ivalid(0) }
  }

  //val ex_rocc_inst = ex_reg_inst.asTypeOf(new RoCCInstruction)
  //val apply_call = ex_ctrl.rocc && ex_rocc_inst.opcode === "b0001011".U && (ex_rocc_inst.funct === 2.U)
  // replay inst in ex stage?
  val ex_pc_valid = ex_reg_valid || ex_reg_replay || ex_reg_xcpt_interrupt
  val wb_dcache_miss = wb_ctrl.mem && !Mux(check_busy && user_mode, true.B, io.dmem.resp.valid)
  val wb_csr_miss    = Mux(isMaster, false.B, Mux(check_busy && user_mode, (wb_ctrl.csr =/= CSR.N && wb_ctrl.wxd) && !check_resp_valid && wb_reg_valid, false.B))
  dontTouch(wb_csr_miss)
  check_s2_nack := (!check_resp_valid && !check_replay_valid && wb_ctrl.mem && wb_reg_valid) || wb_csr_miss
  val replay_ex_structural = ex_ctrl.mem && !io.dmem.req.ready ||
                             ex_ctrl.div && !div.io.req.ready
  dontTouch(replay_ex_structural)
  val replay_ex_load_use = (wb_dcache_miss && ex_reg_load_use)
  val replay_ex = ex_reg_replay || (ex_reg_valid && (replay_ex_structural || replay_ex_load_use))
  val ctrl_killx = take_pc_mem_wb || replay_ex || !ex_reg_valid || wb_flush.need_flush
  // detect 2-cycle load-use delay for LB/LH/SC
  val ex_slow_bypass = ex_ctrl.mem_cmd === M_XSC || ex_reg_mem_size < 2.U
  val ex_sfence = usingVM.B && ex_ctrl.mem && (ex_ctrl.mem_cmd === M_SFENCE || ex_ctrl.mem_cmd === M_HFENCEV || ex_ctrl.mem_cmd === M_HFENCEG)

  val (ex_xcpt, ex_cause) = checkExceptions(List(
    (ex_reg_xcpt_interrupt || ex_reg_xcpt, ex_reg_cause)))

  val exCoverCauses = idCoverCauses
  coverExceptions(ex_xcpt, ex_cause, "EXECUTE", exCoverCauses)

  // memory stage
  val mem_pc_valid = mem_reg_valid || mem_reg_replay || mem_reg_xcpt_interrupt
  val mem_br_target = mem_reg_pc.asSInt +
    Mux(mem_ctrl.branch && mem_br_taken, ImmGen(IMM_SB, mem_reg_inst),
    Mux(mem_ctrl.jal, ImmGen(IMM_UJ, mem_reg_inst),
    Mux(mem_reg_rvc, 2.S, 4.S)))
    dontTouch(mem_br_target)
  //Calculate next-pc, including the jalr instruction
  val mem_npc = (Mux(mem_ctrl.jalr || mem_reg_sfence, encodeVirtualAddress(mem_reg_wdata, mem_reg_wdata).asSInt, mem_br_target) & (-2).S).asUInt
  dontTouch(mem_npc)
  val mem_wrong_npc =
    Mux(ex_pc_valid, mem_npc =/= ex_reg_pc,
    Mux(ibuf.io.inst(0).valid || ibuf.io.imem.valid, mem_npc =/= ibuf.io.pc, true.B))
  val mem_npc_misaligned = !csr.io.status.isa('c'-'a') && mem_npc(1) && !mem_reg_sfence
  val mem_int_wdata = Mux(!mem_reg_xcpt && (mem_ctrl.jalr ^ mem_npc_misaligned), mem_br_target, mem_reg_wdata.asSInt).asUInt
  dontTouch(mem_int_wdata)
  val mem_cfi = mem_ctrl.branch || mem_ctrl.jalr || mem_ctrl.jal
  val mem_cfi_taken = (mem_ctrl.branch && mem_br_taken) || mem_ctrl.jalr || mem_ctrl.jal
  val mem_direction_misprediction = mem_ctrl.branch && mem_br_taken =/= (usingBTB.B && mem_reg_btb_resp.taken)
  val mem_misprediction = if (usingBTB) mem_wrong_npc else mem_cfi_taken
  dontTouch(mem_misprediction)
  take_pc_mem := mem_reg_valid && !mem_reg_xcpt && (mem_misprediction || mem_reg_sfence)

  val mem_rocc_inst = mem_reg_inst.asTypeOf(new RoCCInstruction())
  mem_reg_valid := !ctrl_killx
  mem_reg_replay := !take_pc_mem_wb && replay_ex
  mem_reg_xcpt := !ctrl_killx && ex_xcpt
  mem_reg_xcpt_interrupt := !take_pc_mem_wb && ex_reg_xcpt_interrupt

  // on pipeline flushes, cause mem_npc to hold the sequential npc, which
  // will drive the W-stage npc mux
  when (mem_reg_valid && mem_reg_flush_pipe) {
    mem_reg_sfence := false.B
  }.elsewhen (ex_pc_valid) {
    mem_ctrl := ex_ctrl
    mem_scie_unpipelined := ex_scie_unpipelined
    mem_scie_pipelined := ex_scie_pipelined
    mem_reg_rvc := ex_reg_rvc
    mem_reg_load := ex_ctrl.mem && isRead(ex_ctrl.mem_cmd)
    mem_reg_store := ex_ctrl.mem && isWrite(ex_ctrl.mem_cmd)
    mem_reg_sfence := ex_sfence
    mem_reg_btb_resp := ex_reg_btb_resp
    mem_reg_flush_pipe := ex_reg_flush_pipe
    mem_reg_slow_bypass := ex_slow_bypass
    mem_reg_wphit := ex_reg_wphit

    mem_reg_cause := ex_cause
    mem_reg_inst := ex_reg_inst
    mem_reg_raw_inst := ex_reg_raw_inst
    mem_reg_mem_size := ex_reg_mem_size
    mem_reg_hls_or_dv := Mux(check_busy && user_mode, false.B, io.dmem.req.bits.dv)
    mem_reg_pc := ex_reg_pc
    // IDecode ensured they are 1H  
    when(isMaster){
      mem_reg_wdata := Mux1H(Seq(
                            ex_scie_unpipelined -> ex_scie_unpipelined_wdata,
                            ex_ctrl.zbk         -> ex_zbk_wdata,
                            ex_ctrl.zkn         -> ex_zkn_wdata,
                            ex_ctrl.zks         -> ex_zks_wdata,
                            (!ex_scie_unpipelined && !ex_ctrl.zbk && !ex_ctrl.zkn && !ex_ctrl.zks)
                                                -> alu.io.out,
      ))
    }.otherwise{
      mem_reg_wdata := Mux((ex_ctrl.jalr && ex_reg_inst(12) === 1.U), jump_pc, 
                        Mux1H(Seq(
                            ex_scie_unpipelined -> ex_scie_unpipelined_wdata,
                            ex_ctrl.zbk         -> ex_zbk_wdata,
                            ex_ctrl.zkn         -> ex_zkn_wdata,
                            ex_ctrl.zks         -> ex_zks_wdata,
                            (!ex_scie_unpipelined && !ex_ctrl.zbk && !ex_ctrl.zkn && !ex_ctrl.zks)
                                                -> alu.io.out,
      )))
    }
    
    mem_br_taken := alu.io.cmp_out

    when (ex_ctrl.rxs2 && (ex_ctrl.mem || ex_ctrl.rocc || ex_sfence)) {
      val size = Mux(ex_ctrl.rocc, log2Ceil(xLen/8).U, ex_reg_mem_size)
      mem_reg_rs2 := new StoreGen(size, 0.U, ex_rs(1), coreDataBytes).data
    }
    when (ex_ctrl.jalr && csr.io.status.debug) {
      // flush I$ on D-mode JALR to effect uncached fetch without D$ flush
      mem_ctrl.fence_i := true.B
      mem_reg_flush_pipe := true.B
    }
  }

  val mem_breakpoint = (mem_reg_load && bpu.io.xcpt_ld) || (mem_reg_store && bpu.io.xcpt_st)
  val mem_debug_breakpoint = (mem_reg_load && bpu.io.debug_ld) || (mem_reg_store && bpu.io.debug_st)
  val (mem_ldst_xcpt, mem_ldst_cause) = checkExceptions(List(
    (mem_debug_breakpoint, CSR.debugTriggerCause.U),
    (mem_breakpoint,       Causes.breakpoint.U)))

  val (mem_xcpt, mem_cause) = checkExceptions(List(
    (mem_reg_xcpt_interrupt || mem_reg_xcpt, mem_reg_cause),
    (mem_reg_valid && mem_npc_misaligned,    Causes.misaligned_fetch.U),
    (mem_reg_valid && mem_ldst_xcpt,         mem_ldst_cause)))

  val memCoverCauses = (exCoverCauses ++ List(
    (CSR.debugTriggerCause, "DEBUG_TRIGGER"),
    (Causes.breakpoint, "BREAKPOINT"),
    (Causes.misaligned_fetch, "MISALIGNED_FETCH")
  )).distinct
  coverExceptions(mem_xcpt, mem_cause, "MEMORY", memCoverCauses)

  val dcache_kill_mem = mem_reg_valid && mem_ctrl.wxd && Mux(check_busy && user_mode, false.B, io.dmem.replay_next) // structural hazard on writeback port
  val fpu_kill_mem = mem_reg_valid && mem_ctrl.fp && io.fpu.nack_mem
  val replay_mem  = dcache_kill_mem || mem_reg_replay || fpu_kill_mem
  val killm_common = dcache_kill_mem || take_pc_wb || mem_reg_xcpt || !mem_reg_valid
  div.io.kill := (killm_common || wb_flush.need_take_pc) && RegNext(div.io.req.fire)
  val ctrl_killm = killm_common || mem_xcpt || fpu_kill_mem || wb_flush.need_flush
  
  // writeback stage
  val wb_rocc_inst = wb_reg_inst.asTypeOf(new RoCCInstruction())
  wb_reg_valid := !ctrl_killm
  wb_reg_replay := replay_mem && !take_pc_wb
  wb_reg_xcpt := mem_xcpt && !take_pc_wb
  
                       //|| (isMaster && Mchecke_call) || (isSlave && (instcoun.io.instnum === score_check_left) && CP_end)
  when (mem_pc_valid) {
    wb_ctrl := mem_ctrl
    wb_reg_sfence := mem_reg_sfence
    wb_reg_wdata := Mux(mem_scie_pipelined, mem_scie_pipelined_wdata,
      Mux(!mem_reg_xcpt && mem_ctrl.fp && mem_ctrl.wxd, io.fpu.toint_data, mem_int_wdata))
    when (mem_ctrl.rocc || mem_reg_sfence) {
      wb_reg_rs2 := mem_reg_rs2
    }
    wb_reg_cause := mem_cause
    wb_reg_inst := mem_reg_inst
    wb_reg_raw_inst := mem_reg_raw_inst
    wb_reg_mem_size := mem_reg_mem_size
    wb_reg_hls_or_dv := mem_reg_hls_or_dv
    wb_reg_hfence_v := mem_ctrl.mem_cmd === M_HFENCEV
    wb_reg_hfence_g := mem_ctrl.mem_cmd === M_HFENCEG
    wb_reg_pc := mem_reg_pc
    wb_reg_wphit := mem_reg_wphit | bpu.io.bpwatch.map { bpw => (bpw.rvalid(0) && mem_reg_load) || (bpw.wvalid(0) && mem_reg_store) }

  }

  val (wb_xcpt, wb_cause) = checkExceptions(List(
    (wb_reg_xcpt,  wb_reg_cause),
    (wb_reg_valid && wb_ctrl.mem && Mux(check_busy && user_mode, false.B, io.dmem.s2_xcpt.pf.st), Causes.store_page_fault.U),
    (wb_reg_valid && wb_ctrl.mem && Mux(check_busy && user_mode, false.B, io.dmem.s2_xcpt.pf.ld), Causes.load_page_fault.U),
    (wb_reg_valid && wb_ctrl.mem && Mux(check_busy && user_mode, false.B, io.dmem.s2_xcpt.gf.st), Causes.store_guest_page_fault.U),
    (wb_reg_valid && wb_ctrl.mem && Mux(check_busy && user_mode, false.B, io.dmem.s2_xcpt.gf.ld), Causes.load_guest_page_fault.U),
    (wb_reg_valid && wb_ctrl.mem && Mux(check_busy && user_mode, false.B, io.dmem.s2_xcpt.ae.st), Causes.store_access.U),
    (wb_reg_valid && wb_ctrl.mem && Mux(check_busy && user_mode, false.B, io.dmem.s2_xcpt.ae.ld), Causes.load_access.U),
    (wb_reg_valid && wb_ctrl.mem && Mux(check_busy && user_mode, false.B, io.dmem.s2_xcpt.ma.st), Causes.misaligned_store.U),
    (wb_reg_valid && wb_ctrl.mem && Mux(check_busy && user_mode, false.B, io.dmem.s2_xcpt.ma.ld), Causes.misaligned_load.U)
  ))

  // val (wb_xcpt, wb_cause) = checkExceptions(List(
  //   (wb_reg_xcpt,  wb_reg_cause),
  //   (wb_reg_valid && wb_ctrl.mem && io.dmem.s2_xcpt.pf.st, Causes.store_page_fault.U),
  //   (wb_reg_valid && wb_ctrl.mem && io.dmem.s2_xcpt.pf.ld, Causes.load_page_fault.U),
  //   (wb_reg_valid && wb_ctrl.mem && io.dmem.s2_xcpt.gf.st, Causes.store_guest_page_fault.U),
  //   (wb_reg_valid && wb_ctrl.mem && io.dmem.s2_xcpt.gf.ld, Causes.load_guest_page_fault.U),
  //   (wb_reg_valid && wb_ctrl.mem && io.dmem.s2_xcpt.ae.st, Causes.store_access.U),
  //   (wb_reg_valid && wb_ctrl.mem && io.dmem.s2_xcpt.ae.ld, Causes.load_access.U),
  //   (wb_reg_valid && wb_ctrl.mem && io.dmem.s2_xcpt.ma.st, Causes.misaligned_store.U),
  //   (wb_reg_valid && wb_ctrl.mem && io.dmem.s2_xcpt.ma.ld, Causes.misaligned_load.U)
  // ))

  val wbCoverCauses = List(
    (Causes.misaligned_store, "MISALIGNED_STORE"),
    (Causes.misaligned_load, "MISALIGNED_LOAD"),
    (Causes.store_access, "STORE_ACCESS"),
    (Causes.load_access, "LOAD_ACCESS")
  ) ++ (if(usingVM) List(
    (Causes.store_page_fault, "STORE_PAGE_FAULT"),
    (Causes.load_page_fault, "LOAD_PAGE_FAULT")
  ) else Nil) ++ (if (usingHypervisor) List(
    (Causes.store_guest_page_fault, "STORE_GUEST_PAGE_FAULT"),
    (Causes.load_guest_page_fault, "LOAD_GUEST_PAGE_FAULT"),
  ) else Nil)
  coverExceptions(wb_xcpt, wb_cause, "WRITEBACK", wbCoverCauses)

  val wb_pc_valid = wb_reg_valid || wb_reg_replay || wb_reg_xcpt
  val wb_wxd = wb_reg_valid && wb_ctrl.wxd
  val wb_set_sboard = wb_ctrl.div || wb_dcache_miss || wb_ctrl.rocc
  val replay_wb_common = Mux(check_busy && user_mode, check_s2_nack && !score_check_done, io.dmem.s2_nack) || wb_reg_replay
  val replay_wb_rocc = wb_reg_valid && wb_ctrl.rocc && !io.rocc.cmd.ready

  val let_ret_s_commit = wb_reg_valid && !wb_xcpt && !io.rocc.resp.valid && (wb_reg_pc === score_return_pc)
  val replay_wb = replay_wb_common || replay_wb_rocc || (score_return && !let_ret_s_commit)
  take_pc_wb := replay_wb || wb_xcpt || csr.io.eret || wb_reg_flush_pipe || (RegNext(wb_flush.need_take_pc) && user_mode)
  
  // writeback arbitration
  val dmem_resp_xpu     = Mux(check_busy, check_resp_xpu  , !io.dmem.resp.bits.tag(0).asBool                 )
  val dmem_resp_fpu     = Mux(check_busy, check_resp_fpu  , io.dmem.resp.bits.tag(0).asBool                  )
  val dmem_resp_waddr   = Mux(check_busy, check_resp_waddr, io.dmem.resp.bits.tag(5, 1)                      )
  val dmem_resp_valid   = Mux(check_busy, (check_resp_valid || check_replay_valid) && (wb_ctrl.mem_cmd.isOneOf(M_XRD, M_XLR, M_XSC) || isAMO(wb_ctrl.mem_cmd)), io.dmem.resp.valid && io.dmem.resp.bits.has_data )
  val dmem_resp_replay  = Mux(check_busy, false.B         , dmem_resp_valid && io.dmem.resp.bits.replay      )

  div.io.resp.ready := !wb_wxd
  val ll_wdata = WireDefault(div.io.resp.bits.data)
  val ll_waddr = WireDefault(div.io.resp.bits.tag)
  val ll_wen = WireDefault(div.io.resp.fire && Mux(sbo(32) || fp_sbo(32), true.B, Mux(user_mode, (!mcore_check_done && !score_check_done && !(check_busy && SFIFO_almostempty)), true.B)))
  if (usingRoCC) {
    io.rocc.resp.ready := !wb_wxd
    when (io.rocc.resp.fire) {
      div.io.resp.ready := false.B
      ll_wdata := io.rocc.resp.bits.data
      ll_waddr := io.rocc.resp.bits.rd
      ll_wen := true.B && Mux(user_mode, (!mcore_check_done && !score_check_done && !(check_busy && SFIFO_almostempty)), true.B)
    }
  }
  when (dmem_resp_replay && dmem_resp_xpu) {
    div.io.resp.ready := false.B
    if (usingRoCC)
      io.rocc.resp.ready := false.B
    ll_waddr := dmem_resp_waddr
    ll_wen := true.B && Mux(user_mode, (!mcore_check_done && !score_check_done && !(check_busy && SFIFO_almostempty)), true.B)
  }

  val wb_valid = wb_reg_valid && !replay_wb && !wb_xcpt
  val wb_wen = wb_valid && wb_ctrl.wxd
  val rf_wen = Mux(receiving_rf || score_return_rf, rf_apply_en, wb_wen || ll_wen)
  val rf_waddr = Mux(receiving_rf || score_return_rf, rf_apply_addr, Mux(ll_wen, ll_waddr, wb_waddr))
  val rf_wdata = Mux(receiving_rf || score_return_rf, rf_apply_data,
                    Mux(dmem_resp_valid && dmem_resp_xpu && !mcore_check_done, Mux(check_busy && user_mode, check_resp_data_ld, Mux(sc_undone && user_mode, sc_cond, io.dmem.resp.bits.data(xLen-1, 0))),
                      Mux(ll_wen, ll_wdata,
                        Mux(wb_ctrl.csr =/= CSR.N, Mux(check_busy && user_mode, check_resp_data_ld, csr.io.rw.rdata),
                            Mux(wb_ctrl.mul, mul.map(_.io.resp.bits.data).getOrElse(wb_reg_wdata),
                                wb_reg_wdata)))))


  //check_resp_valid := RegNext(mem_reg_valid && (mem_ctrl.mem || (mem_ctrl.csr =/= CSR.N && mem_ctrl.wxd)) && check_req_ready)
  check_resp_valid := Mux(RegNext(!(killm_common || mem_xcpt || fpu_kill_mem || replay_wb)), RegNext(mem_reg_valid && (mem_ctrl.mem || (mem_ctrl.csr =/= CSR.N && mem_ctrl.wxd)) && check_req_ready), false.B)
  check_resp_replay := !wb_valid && wb_ctrl.mem && check_resp_valid
  when(check_resp_replay){
    check_replay_inst := wb_reg_inst
    check_replay_flag := true.B
  }
  check_replay_ready  := mem_reg_valid && mem_ctrl.mem && (check_replay_inst === mem_reg_inst)
  check_replay_valid  := Mux(RegNext(!(killm_common || mem_xcpt || fpu_kill_mem || replay_wb)), RegNext(mem_reg_valid && mem_ctrl.mem && check_replay_ready), false.B)
  //check_resp_valid_test := RegEnable(mem_reg_valid && mem_ctrl.mem && check_req_ready, !(killm_common || mem_ldst_xcpt || fpu_kill_mem || replay_wb))
  //check_resp_valid_test := Mux(RegNext(!(killm_common || mem_ldst_xcpt || fpu_kill_mem || replay_wb)), RegNext(mem_reg_valid && (mem_ctrl.mem || (mem_ctrl.csr =/= CSR.N && mem_ctrl.wxd)) && check_req_ready), false.B)

  
  wb_reg_flush_pipe := (!ctrl_killm && mem_reg_flush_pipe)
  custom_jalr := wb_valid && wb_ctrl.jalr && wb_reg_inst(12) === 1.U && isSlave
  score_check_overtaking := SFIFO_almostempty && wb_valid && check_busy && user_mode
  when(custom_jalr){
    have_jumped := true.B
  }

  when(wb_valid){
    check_resp_waddr := wb_waddr
    check_replay_flag := false.B
    //comit_ctrl       := wb_ctrl
  }
  dontTouch(check_resp_waddr)
  dontTouch(wb_waddr)

  val wb_npc = RegEnable(mem_npc, mem_reg_valid)
  when(csr.io.eret){
    imem_req_pc := csr.io.evec
  }.elsewhen(wb_valid){
    imem_req_pc := wb_npc
  }
  dontTouch(imem_req_pc)
  //wb_flush signals
  wb_flush.m_flush.check_done    := instcoun.io.check_done && isMaster
  wb_flush.m_flush.complete      := instcoun.io.mcore_check_complete && isMaster
  wb_flush.s_flush.complete      := instcoun.io.score_check_complete && isSlave
  wb_flush.s_flush.overtakeing   := score_check_overtaking && isSlave
  wb_flush.s_flush.custom_jalr   := custom_jalr && isSlave
  wb_flush.s_flush.check_done    := instcoun.io.check_done && isSlave
  dontTouch(wb_flush)
  /*
  val rf_wdata1 = Mux(isMaster, Mux(dmem_resp_valid && dmem_resp_xpu, io.dmem.resp.bits.data(xLen-1, 0),
                                    Mux(ll_wen, ll_wdata,
                                    Mux(wb_ctrl.csr =/= CSR.N, csr.io.rw.rdata,
                                    Mux(wb_ctrl.mul, mul.map(_.io.resp.bits.data).getOrElse(wb_reg_wdata),
                                    wb_reg_wdata)))),
                                Mux(dmem_resp_valid && dmem_resp_xpu && wb_ctrl.mem_cmd === M_XRD, Mux(check_busy, check_resp_data, io.dmem.resp.bits.data(xLen-1, 0)),
                                    Mux(ll_wen, ll_wdata,
                                    Mux(wb_ctrl.csr =/= CSR.N, csr.io.rw.rdata,
                                    Mux(wb_ctrl.mul, mul.map(_.io.resp.bits.data).getOrElse(wb_reg_wdata),
                                    wb_reg_wdata)))))    
                                    */           

  dontTouch(rf_wdata)
  Mchecke_call   := RegNext(wb_ctrl.rocc && wb_valid && wb_rocc_inst.opcode === "b0001011".U && (wb_rocc_inst.funct === 11.U) && isMaster)
  Mcheck_end     := wb_ctrl.rocc && wb_valid && wb_rocc_inst.opcode === "b0001011".U && (wb_rocc_inst.funct === 11.U) && isMaster
  Mcheck_call    := wb_ctrl.rocc && wb_valid && wb_rocc_inst.opcode === "b0001011".U && (wb_rocc_inst.funct === 3.U)
  //Mcheck_end     := !mem_misprediction && mem_ctrl.rocc && mem_reg_valid && mem_rocc_inst.opcode === "b0001011".U && (mem_rocc_inst.funct === 11.U) && isMaster
  score_apply    := wb_ctrl.rocc && wb_valid && wb_rocc_inst.opcode === "b0001011".U && (wb_rocc_inst.funct === 2.U)
  Srecode_call   := wb_ctrl.rocc && wb_valid && wb_rocc_inst.opcode === "b0001011".U && (wb_rocc_inst.funct === 9.U)
  when(wb_valid && wb_ctrl.rocc && wb_rocc_inst.opcode === "b0001011".U && wb_rocc_inst.funct === 2.U){
    slave_rece_en := io.rocc.cmd.bits.rs1(0).asBool
  }
  when(wb_valid && wb_ctrl.rocc && wb_rocc_inst.opcode === "b0001011".U && wb_rocc_inst.funct === 12.U){
    check_instnum_threshold := io.rocc.cmd.bits.rs1
  }
  when(score_apply){
    have_reve := true.B
  }
  //tw's customs
  q_copyvalid := copyvalid
  dontTouch(copyvalid)
  dontTouch(q_copyvalid)
  
  //instruction  counter IO hook up
  instcoun.io.wb_valid := wb_valid
  instcoun.io.start := (mcore_checking && isMaster) || (check_busy && user_mode && have_jumped && isSlave)

  //copy signals connect
  copy_signals.Mcheck_call         := Mcheck_call && isMaster && slave_is_umode
  copy_signals.Mchecke_call        := Mchecke_call && isMaster && mcore_checking
  copy_signals.common_endCP        := instcoun.io.copy_valid && isMaster
  copy_signals.common_startCP      := mcore_check_free && start_check && !mcore_checking && !rf_sentvalid && slave_is_umode && user_mode && !csr.io.interrupt && !sentdone_butuncheck && (ppn_trace_original === ppn_trace)
  copy_signals.task_change_startCP := false.B //task_monitor.master_can_startcheck && !mcore_checking && start_check && !rf_sentvalid && slave_is_umode
  copy_signals.task_change_endCP   := task_monitor.master_need_endcheck && mcore_checking && !rf_sentvalid
  copy_signals.Srecode_call        := Srecode_call && isSlave

  when(copy_signals.copy){
    copyvalid := true.B
    timer_counter := csr.io.time(31, 0)
  }

  when(copy_signals.common_startCP){
    start_copyvalid := true.B
  }

  when(start_copyvalid){
    jump_pc := intpc_reg
  }

  //when(Mchecke_call){
  //  end_check := true.B
  //}

  when(instcoun.io.check_done){
    score_check_done := true.B && isSlave
    mcore_check_done := true.B && isMaster
  }

  when(instcoun.io.check_done_common){
    mcore_check_done_common := true.B && isMaster
  }

  intpc_reg := Mux(csr.io.eret && !csr.io.trace(0).exception, csr.io.evec, Mux(wb_valid, wb_npc, intpc_reg)) 
  

  //copy and sent module IO hook up  
  for( i <- 1 until 32){
    isa.io.intreg_input(i.U) := rf.read(i.U)
  }
  isa.io.intreg_input(0) := 0.U
  for( i <-0 until 32){
    isa.io.fpreg_input(i) := io.fpu.frf(i)
  }
  
  isa.io.intpc_input := intpc_reg
  isa.io.fpcsr_input := csr.io.fcsr_read
  score_return_pc    := RegEnable(intpc_reg, en_copyvalid && isSlave) 
  
  instcoun.io.copy_done         := isa.io.copy_done
  instcoun.io.inst_left         := score_check_left
  instcoun.io.CP_end            := CP_end && isSlave
  instcoun.io.Mchecke_call      := Mcheck_end
  instcoun.io.Schecke_call      := Scheck_end && isSlave
  instcoun.io.instnum_threshold := check_instnum_threshold
  instcoun.io.user_mode         := user_mode
  instcoun.io.CP_mid            := CP_mid_ofchange && isSlave
  instcoun.io.mcore_endcheck    := task_monitor.slave_priv_change_tonotuser && mcore_checking && !rf_sentvalid && user_mode
  instcoun.io.mode_signc        := mode_signc
  instcoun.io.inst_left_wire    := inst_left
  instcoun.io.isMaster          := isMaster

  when(ctrl_killd && q_copyvalid && fsign){
    pc_reg := ibuf.io.pc
    fsign := false.B
  }
  
  when(isa.io.copy_done){
    rf_sentvalid := true.B
    fsign := true.B
  }
  when(isa.io.copy_done || (q_copyvalid && !mcore_checking && !user_mode)){
    copyvalid := false.B
    start_copyvalid := false.B
  }

  when(score_return){
    score_return_rf := true.B
  }
  isa.io.sent_valid := ((rf_sentvalid && isMaster && !FIFO.io.full) || (score_return_rf && isSlave))
  when(isa.io.sent_done){
    rf_sentvalid            := false.B
    score_return_rf         := false.B
    mcore_check_done        := false.B
    mcore_check_done_common := false.B
  }


  val out_data       = isa.io.sent_output
  val out_sign       = out_data(255, 253) === "b101".U
  val out_datatype   = out_data(252, 251)
  val out_idx        = out_data(250, 246)
  val out_wrf_data   = out_data(63, 0)
  val out_wfprf_data = out_data(127, 64)
  dontTouch(out_data)
  dontTouch(out_sign)
  dontTouch(out_datatype)
  dontTouch(out_idx)
  dontTouch(out_wrf_data)
  when(out_sign && isSlave){
    // when(out_datatype === 1.U){
    //   rf_apply_en   := true.B
    //   rf_apply_addr := out_idx
    //   rf_apply_data := out_wrf_data
    //   fprf_data    := out_wfprf_data
    //   fprf_idx     := out_idx
    //   fp_apply_en  := true.B
    // }
    when(out_datatype === 0.U){
      fcsr_out      := out_data(7, 0)
      csr_apply_en  := score_return_rf
    }.elsewhen(out_datatype === 1.U){
      rf_apply_en   := true.B
      rf_apply_addr := out_idx
      rf_apply_data := out_wrf_data
      fprf_data    := out_wfprf_data
      fprf_idx     := out_idx
      fp_apply_en  := true.B
    }
  }


  //use the register to test
  val testreg = RegInit(0.U(65.W))
  testreg := isa.io.sent_output
  dontTouch(testreg)
  //tw's custome
  rf_read_data := rf.read(rf_read_addr)

  when (rf_wen) { rf.write(rf_waddr, rf_wdata) }

  // hook up control/status regfile
  csr.io.fcsr_in := fcsr_out
  csr.io.pfcsr_en := csr_apply_en
  csr.io.ungated_clock := clock
  csr.io.decode(0).inst := id_inst(0)
  csr.io.exception := wb_xcpt
  csr.io.cause := wb_cause
  csr.io.retire := wb_valid
  csr.io.inst(0) := (if (usingCompressed) Cat(Mux(wb_reg_raw_inst(1, 0).andR, wb_reg_inst >> 16, 0.U), wb_reg_raw_inst(15, 0)) else wb_reg_inst)
  csr.io.interrupts := io.interrupts
  csr.io.hartid := io.hartid
  io.fpu.fcsr_rm := csr.io.fcsr_rm
  csr.io.fcsr_flags := io.fpu.fcsr_flags
  io.fpu.time := csr.io.time(31,0)
  io.fpu.hartid := io.hartid
  csr.io.rocc_interrupt := io.rocc.interrupt
  csr.io.pc := Mux(score_check_done && user_mode, score_return_pc, wb_reg_pc)
  val tval_dmem_addr = !wb_reg_xcpt
  val tval_any_addr = tval_dmem_addr ||
    wb_reg_cause.isOneOf(Causes.breakpoint.U, Causes.fetch_access.U, Causes.fetch_page_fault.U, Causes.fetch_guest_page_fault.U)
  val tval_inst = wb_reg_cause === Causes.illegal_instruction.U
  val tval_valid = wb_xcpt && (tval_any_addr || tval_inst)
  csr.io.gva := wb_xcpt && (tval_any_addr && csr.io.status.v || tval_dmem_addr && wb_reg_hls_or_dv)
  csr.io.tval := Mux(tval_valid, encodeVirtualAddress(wb_reg_wdata, wb_reg_wdata), 0.U)
  csr.io.htval := {
    val htval_valid_imem = wb_reg_xcpt && wb_reg_cause === Causes.fetch_guest_page_fault.U
    val htval_imem = Mux(htval_valid_imem, io.imem.gpa.bits, 0.U)
    assert(!htval_valid_imem || io.imem.gpa.valid)

    val htval_valid_dmem = wb_xcpt && tval_dmem_addr && io.dmem.s2_xcpt.gf.asUInt.orR && !io.dmem.s2_xcpt.pf.asUInt.orR //Mux(check_busy, false.B, io.dmem.s2_xcpt.gf.asUInt.orR) && !Mux(check_busy, false.B, io.dmem.s2_xcpt.pf.asUInt.orR)
    val htval_dmem = Mux(htval_valid_dmem, io.dmem.s2_gpa, 0.U)
    // val htval_valid_dmem = wb_xcpt && tval_dmem_addr && Mux(check_busy, false.B, io.dmem.s2_xcpt.gf.asUInt.orR) && !Mux(check_busy, false.B, io.dmem.s2_xcpt.pf.asUInt.orR)
    // val htval_dmem = Mux(htval_valid_dmem, Mux(check_busy, 0.U, io.dmem.s2_gpa), 0.U)

    (htval_dmem | htval_imem) >> hypervisorExtraAddrBits
  }
  ppn_trace                                := csr.io.ptbr.ppn
  task_monitor.trace_ppn                   := csr.io.ptbr.ppn
  task_monitor.trace_priv                  := csr.io.status.prv
  task_monitor.trace_slave_priv_umode      := slave_is_umode
  task_monitor.trace_slave_priv_umode_next := RegNext(slave_is_umode)
  task_monitor.trace_exception             := RegNext(csr.io.trace(0).exception)
  task_monitor.xcpt_cause                  := RegNext(csr.io.trace(0).cause & (~((BigInt(1) << 63).U)))
  task_monitor.trace_ppn_next              := RegNext(csr.io.ptbr.ppn)
  task_monitor.trace_priv_next             := RegNext(csr.io.status.prv)
  dontTouch(task_monitor)

  when(start_check && task_monitor.ppn_change){
    ppn_change_count := ppn_change_count + 1.U
  }

  when(start_check && ppn_change_count === 0.U){
    ppn_trace_original := task_monitor.trace_ppn
  }

  io.ptw.ptbr := csr.io.ptbr
  io.ptw.hgatp := csr.io.hgatp
  io.ptw.vsatp := csr.io.vsatp
  (io.ptw.customCSRs.csrs zip csr.io.customCSRs).map { case (lhs, rhs) => lhs := rhs }
  io.ptw.status := csr.io.status
  io.ptw.hstatus := csr.io.hstatus
  io.ptw.gstatus := csr.io.gstatus
  io.ptw.pmp := csr.io.pmp
  csr.io.rw.addr := wb_reg_inst(31,20)
  csr.io.rw.cmd := CSR.maskCmd(wb_reg_valid, wb_ctrl.csr)
  csr.io.rw.wdata := wb_reg_wdata
  io.trace.insns := csr.io.trace
  io.trace.time := csr.io.time
  io.rocc.csrs := csr.io.roccCSRs
  if (rocketParams.debugROB) {
    val csr_trace_with_wdata = WireInit(csr.io.trace(0))
    csr_trace_with_wdata.wdata.get := rf_wdata
    DebugROB.pushTrace(clock, reset,
      io.hartid, csr_trace_with_wdata,
      (wb_ctrl.wfd || (wb_ctrl.wxd && wb_waddr =/= 0.U)) && !csr.io.trace(0).exception,
      wb_ctrl.wxd && wb_wen && !wb_set_sboard,
      wb_waddr + Mux(wb_ctrl.wfd, 32.U, 0.U))

    io.trace.insns(0) := DebugROB.popTrace(clock, reset, io.hartid)

    DebugROB.pushWb(clock, reset, io.hartid, ll_wen, rf_waddr, rf_wdata)
  }

  for (((iobpw, wphit), bp) <- io.bpwatch zip wb_reg_wphit zip csr.io.bp) {
    iobpw.valid(0) := wphit
    iobpw.action := bp.control.action
  }

  val hazard_targets = Seq((id_ctrl.rxs1 && id_raddr1 =/= 0.U, id_raddr1),
                           (id_ctrl.rxs2 && id_raddr2 =/= 0.U, id_raddr2),
                           (id_ctrl.wxd  && id_waddr  =/= 0.U, id_waddr))
  val fp_hazard_targets = Seq((io.fpu.dec.ren1, id_raddr1),
                              (io.fpu.dec.ren2, id_raddr2),
                              (io.fpu.dec.ren3, id_raddr3),
                              (io.fpu.dec.wen, id_waddr))

  val sboard = new Scoreboard(32, true)
  sboard.clear(ll_wen, ll_waddr)
  def id_sboard_clear_bypass(r: UInt) = {
    // ll_waddr arrives late when D$ has ECC, so reshuffle the hazard check
    if (!tileParams.dcache.get.dataECC.isDefined) ll_wen && ll_waddr === r
    else div.io.resp.fire && div.io.resp.bits.tag === r || dmem_resp_replay && dmem_resp_xpu && dmem_resp_waddr === r
  }
  val id_sboard_hazard = checkHazards(hazard_targets, rd => sboard.read(rd) && !id_sboard_clear_bypass(rd))
  sboard.set(wb_set_sboard && wb_wen, wb_waddr)

  // stall for RAW/WAW hazards on CSRs, loads, AMOs, and mul/div in execute stage.
  val ex_cannot_bypass = ex_ctrl.csr =/= CSR.N || ex_ctrl.jalr || ex_ctrl.mem || ex_ctrl.mul || ex_ctrl.div || ex_ctrl.fp || ex_ctrl.rocc || ex_scie_pipelined
  val data_hazard_ex = ex_ctrl.wxd && checkHazards(hazard_targets, _ === ex_waddr)
  val fp_data_hazard_ex = id_ctrl.fp && ex_ctrl.wfd && checkHazards(fp_hazard_targets, _ === ex_waddr)
  val id_ex_hazard = ex_reg_valid && (data_hazard_ex && ex_cannot_bypass || fp_data_hazard_ex)

  // stall for RAW/WAW hazards on CSRs, LB/LH, and mul/div in memory stage.
  val mem_mem_cmd_bh =
    if (fastLoadWord) (!fastLoadByte).B && mem_reg_slow_bypass
    else true.B
  val mem_cannot_bypass = mem_ctrl.csr =/= CSR.N || mem_ctrl.mem && mem_mem_cmd_bh || mem_ctrl.mul || mem_ctrl.div || mem_ctrl.fp || mem_ctrl.rocc
  val data_hazard_mem = mem_ctrl.wxd && checkHazards(hazard_targets, _ === mem_waddr)
  val fp_data_hazard_mem = id_ctrl.fp && mem_ctrl.wfd && checkHazards(fp_hazard_targets, _ === mem_waddr)
  val id_mem_hazard = mem_reg_valid && (data_hazard_mem && mem_cannot_bypass || fp_data_hazard_mem)
  id_load_use := mem_reg_valid && data_hazard_mem && mem_ctrl.mem

  // stall for RAW/WAW hazards on load/AMO misses and mul/div in writeback.
  val data_hazard_wb = wb_ctrl.wxd && checkHazards(hazard_targets, _ === wb_waddr)
  val fp_data_hazard_wb = id_ctrl.fp && wb_ctrl.wfd && checkHazards(fp_hazard_targets, _ === wb_waddr)
  val id_wb_hazard = wb_reg_valid && (data_hazard_wb && wb_set_sboard || fp_data_hazard_wb)

  val id_stall_fpu = if (usingFPU) {
    val fp_sboard = new Scoreboard(32)
    fp_sboard.set((wb_dcache_miss && wb_ctrl.wfd || io.fpu.sboard_set) && wb_valid, wb_waddr)
    fp_sboard.clear(dmem_resp_replay && dmem_resp_fpu, dmem_resp_waddr)
    fp_sboard.clear(io.fpu.sboard_clr, io.fpu.sboard_clra)

    //tw's customs
    fp_sbo(0) := false.B
    for(i <- 1 until 33){ 
      fp_sbo(i) := fp_sbo(i-1) || fp_sboard.read(i.U) 
    }
    //tw's custome

    checkHazards(fp_hazard_targets, fp_sboard.read _)
  } else false.B

  val dcache_blocked = {
    // speculate that a blocked D$ will unblock the cycle after a Grant
    val blocked = Reg(Bool())
    blocked := !io.dmem.req.ready && io.dmem.clock_enabled && !io.dmem.perf.grant && (blocked || io.dmem.req.valid || io.dmem.s2_nack)
    blocked && !io.dmem.perf.grant
  }
  val rocc_blocked = Reg(Bool())
  rocc_blocked := !wb_xcpt && !io.rocc.cmd.ready && (io.rocc.cmd.valid || rocc_blocked)
  
  dontTouch(id_ctrl.mem)
  val ctrl_stalld =
    id_ex_hazard || id_mem_hazard || id_wb_hazard || id_sboard_hazard ||
    csr.io.singleStep && (ex_reg_valid || mem_reg_valid || wb_reg_valid) ||
    id_csr_en && csr.io.decode(0).fp_csr && !io.fpu.fcsr_rdy ||
    id_ctrl.fp && id_stall_fpu ||
    id_ctrl.mem && dcache_blocked ||
    //id_ctrl.mem && dcache_blocked || // reduce activity during D$ misses
    Mux(isMaster, id_ctrl.mem && dcache_blocked, Mux(check_busy && user_mode, mem_ctrl.mem && !check_req_ready && !check_replay_ready && mem_reg_valid, id_ctrl.mem && dcache_blocked)) ||
    //Mux(isMaster, false.B, Mux(check_busy && user_mode, (mem_ctrl.csr =/= CSR.N && mem_ctrl.wxd) && !check_req_ready && mem_reg_valid, false.B)) || 
    //(ex_ctrl.mem && check_busy && !check_req_ready && isSlave && ex_reg_valid) ||
    id_ctrl.rocc && rocc_blocked || // reduce activity while RoCC is busy
    id_ctrl.div && (!(div.io.req.ready || (div.io.resp.valid && !wb_wxd)) || div.io.req.valid) || // reduce odds of replay
    !clock_en ||
    id_do_fence ||
    csr.io.csr_stall ||
    id_reg_pause ||
    io.traceStall ||
    q_copyvalid ||
    (receiving_rf || rece_rf_done) ||
    (rf_sentvalid && isMaster && Mux(user_mode, true.B, !FIFO.io.full)) ||
    (score_return_rf && isSlave) ||
    //(apply_call && !rece_rf_done) ||
    (MFIFO_almostfull && mcore_checking) ||
    (SFIFO_almostempty && (check_busy || receiving_rf || rece_rf_done)) ||
    (score_check_done && isSlave && user_mode) ||
    (past_sign && isSlave)


  dontTouch(ctrl_stalld)
  ctrl_killd := !ibuf.io.inst(0).valid || ibuf.io.inst(0).bits.replay || take_pc_mem_wb || ctrl_stalld || csr.io.interrupt || wb_flush.need_flush
  
  io.imem.req.valid := take_pc
  io.imem.req.bits.speculative := !take_pc_wb
  io.imem.req.bits.pc :=
    Mux(wb_xcpt || (csr.io.eret/* && !(score_check_done && score_comp_done && user_mode)*/), csr.io.evec, // exception or [m|s]ret
    Mux(replay_wb,  Mux(score_return, score_return_pc, wb_reg_pc),   // replay
                    Mux(((RegNext(wb_flush.need_replay_pc))), imem_req_pc, 
                    Mux(RegNext(wb_flush.need_jump), jump_pc, mem_npc))))   // flush or branch misprediction
  // for now just ignore fence.i
  io.imem.flush_icache := Mux(check_busy && user_mode, false.B, wb_reg_valid && wb_ctrl.fence_i && !io.dmem.s2_nack)
  io.imem.might_request := {
    imem_might_request_reg := ex_pc_valid || mem_pc_valid || io.ptw.customCSRs.disableICacheClockGate || true.B
    imem_might_request_reg
  }
  io.imem.progress := RegNext(wb_reg_valid && !replay_wb_common)
  io.imem.sfence.valid := wb_reg_valid && wb_reg_sfence
  io.imem.sfence.bits.rs1 := wb_reg_mem_size(0)
  io.imem.sfence.bits.rs2 := wb_reg_mem_size(1)
  io.imem.sfence.bits.addr := wb_reg_wdata
  io.imem.sfence.bits.asid := wb_reg_rs2
  io.imem.sfence.bits.hv := wb_reg_hfence_v
  io.imem.sfence.bits.hg := wb_reg_hfence_g
  io.ptw.sfence := io.imem.sfence

  ibuf.io.inst(0).ready := !ctrl_stalld

  io.imem.btb_update.valid := mem_reg_valid && !take_pc_wb && mem_wrong_npc && (!mem_cfi || mem_cfi_taken)
  io.imem.btb_update.bits.isValid := mem_cfi
  io.imem.btb_update.bits.cfiType :=
    Mux((mem_ctrl.jal || mem_ctrl.jalr) && mem_waddr(0), CFIType.call,
    Mux(mem_ctrl.jalr && (mem_reg_inst(19,15) & regAddrMask.U) === BitPat("b00?01"), CFIType.ret,
    Mux(mem_ctrl.jal || mem_ctrl.jalr, CFIType.jump,
    CFIType.branch)))
  io.imem.btb_update.bits.target := io.imem.req.bits.pc
  io.imem.btb_update.bits.br_pc := (if (usingCompressed) mem_reg_pc + Mux(mem_reg_rvc, 0.U, 2.U) else mem_reg_pc)
  io.imem.btb_update.bits.pc := ~(~io.imem.btb_update.bits.br_pc | (coreInstBytes*fetchWidth-1).U)
  io.imem.btb_update.bits.prediction := mem_reg_btb_resp

  io.imem.bht_update.valid := mem_reg_valid && !take_pc_wb
  io.imem.bht_update.bits.pc := io.imem.btb_update.bits.pc
  io.imem.bht_update.bits.taken := mem_br_taken
  io.imem.bht_update.bits.mispredict := mem_wrong_npc
  io.imem.bht_update.bits.branch := mem_ctrl.branch
  io.imem.bht_update.bits.prediction := mem_reg_btb_resp.bht

  io.fpu.valid := !ctrl_killd && id_ctrl.fp
  io.fpu.killx := ctrl_killx
  io.fpu.killm := killm_common || wb_flush.need_flush
  io.fpu.inst := id_inst(0)
  io.fpu.fromint_data := ex_rs(0)
  io.fpu.dmem_resp_val := Mux(isMaster, dmem_resp_valid && dmem_resp_fpu && !RegNext(wb_flush.m_flush.check_done || wb_flush.m_flush.complete), dmem_resp_valid && dmem_resp_fpu)
  io.fpu.dmem_resp_data := Mux(check_busy && user_mode, check_resp_data_ld, (if (minFLen == 32) io.dmem.resp.bits.data_word_bypass else io.dmem.resp.bits.data))
  io.fpu.dmem_resp_type := Mux(check_busy && user_mode, check_resp_size, io.dmem.resp.bits.size)
  io.fpu.dmem_resp_tag := dmem_resp_waddr
  io.fpu.keep_clock_enabled := io.ptw.customCSRs.disableCoreClockGate
  io.fpu.apply_en := fp_apply_en
  io.fpu.apply_bits := fprf_data
  io.fpu.apply_idx := fprf_idx

  io.dmem.req.valid     := Mux(check_busy && user_mode, false.B, ex_reg_valid && ex_ctrl.mem)
  val ex_dcache_tag = Cat(ex_waddr, ex_ctrl.fp)
  require(coreParams.dcacheReqTagBits >= ex_dcache_tag.getWidth)
  io.dmem.req.bits.tag              := Mux(check_busy && user_mode, 0.U, ex_dcache_tag)
  io.dmem.req.bits.cmd              := Mux(check_busy && user_mode, 0.U, ex_ctrl.mem_cmd)
  io.dmem.req.bits.size             := Mux(check_busy && user_mode, 0.U, ex_reg_mem_size)
  io.dmem.req.bits.signed           := Mux(check_busy && user_mode, false.B, !Mux(ex_reg_hls, ex_reg_inst(20), ex_reg_inst(14)))
  io.dmem.req.bits.phys             := Mux(check_busy && user_mode, false.B, false.B)
  io.dmem.req.bits.addr             := Mux(check_busy && user_mode, 0.U, encodeVirtualAddress(ex_rs(0), alu.io.adder_out))
  io.dmem.req.bits.idx.foreach(_    := Mux(check_busy && user_mode, 0.U, io.dmem.req.bits.addr))
  io.dmem.req.bits.dprv             := Mux(check_busy && user_mode, 0.U, Mux(ex_reg_hls, csr.io.hstatus.spvp, csr.io.status.dprv))
  io.dmem.req.bits.dv               := Mux(check_busy && user_mode, false.B, ex_reg_hls || csr.io.status.dv)
  io.dmem.s1_data.data              := Mux(check_busy && user_mode, 0.U, (if (fLen == 0) mem_reg_rs2 else Mux(mem_ctrl.fp, Fill((xLen max fLen) / fLen, io.fpu.store_data), mem_reg_rs2)))
  io.dmem.s1_kill                   := Mux(check_busy && user_mode, false.B, killm_common || mem_ldst_xcpt || fpu_kill_mem || wb_flush.need_jump)
  io.dmem.s2_kill                   := false.B
  // don't let D$ go to sleep if we're probably going to use it soon
  io.dmem.keep_clock_enabled        := ibuf.io.inst(0).valid && id_ctrl.mem && !csr.io.csr_stall

  io.rocc.cmd.valid := wb_reg_valid && wb_ctrl.rocc && !replay_wb_common
  io.rocc.exception := wb_xcpt && csr.io.status.xs.orR
  io.rocc.cmd.bits.status := csr.io.status
  io.rocc.cmd.bits.inst := wb_reg_inst.asTypeOf(new RoCCInstruction())
  io.rocc.cmd.bits.rs1 := wb_reg_wdata
  io.rocc.cmd.bits.rs2 := wb_reg_rs2       
  io.rocc.score_rece_done := rece_rf_done 
  io.rocc.score_recerf := rf_ready
  io.rocc.mcore_runing := !ctrl_killd 
  io.rocc.score_checkmode := check_mode && isSlave                          

  //rocc.resp.data
  val NumID = RegInit(VecInit(Seq.fill(2)(0.U(4.W))))
  val tempID = RegInit(VecInit(Seq.fill(GlobalParams.Num_Groupcores)(15.U(4.W))))
  when(io.rocc.cmd.valid && io.rocc.cmd.bits.inst.opcode === "b0001011".U && io.rocc.cmd.bits.inst.funct === 1.U){
      custom_regbool := io.rocc.cmd.bits.rs1(0).asBool
  }
  
  
  

  //when(HartID1.contains(io.hartid)){
  when(io.rocc.cmd.valid && io.rocc.cmd.bits.inst.opcode === "b0001011".U && io.rocc.cmd.bits.inst.funct === 4.U){
    NumMaster := io.rocc.cmd.bits.rs1(7, 4)
    NumSlave := GlobalParams.Num_Groupcores.U - io.rocc.cmd.bits.rs1(7, 4)
    tempID := VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){i => io.rocc.cmd.bits.rs2(4 * (i + 1) - 1, 4 * i)})
  }
  val nexttempID = RegNext(RegNext(tempID, VecInit(Seq.fill(GlobalParams.Num_Groupcores)(15.U(4.W)))))
  
  val resp_funct = RegNext(RegNext(RegNext(io.rocc.cmd.bits.inst.funct)))
  
  when(io.rocc.resp.valid && io.rocc.resp.bits.opcode === "b0001011".U && io.rocc.resp.bits.funct === 4.U){ //Ensure the ID and select signals synchronous changes
    for(i <- 0 until GlobalParams.Num_Groupcores){
      when(i.U < NumMaster){
        MasterID(i) := nexttempID(GlobalParams.Num_Groupcores - i - 1)
        SlaveID(i) := 15.U
      }.otherwise{
        MasterID(i) := 15.U
        SlaveID(i) := nexttempID(GlobalParams.Num_Groupcores - i - 1)
      }
    }
    reg_sels := VecInit((VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){i => io.rocc.resp.bits.data(4 * (i + 1) - 1, 4 * i)})).map(k => VecInit(k.asBools)))
    reg_slavesels := VecInit((0 until 4).map {i =>
    Reverse(Cat(VecInit(((VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){i => io.rocc.resp.bits.data(4 * (i + 1) - 1, 4 * i)})).map(k => VecInit(k.asBools)))).map(_(i))))
    })
  }
  //}
  // .otherwise{
  //   when(io.rocc.cmd.valid && io.rocc.cmd.bits.inst.opcode === "b0001011".U && io.rocc.cmd.bits.inst.funct === 4.U){
  //     NumMaster := io.rocc.cmd.bits.rs1(15, 12)
  //     NumSlave := GlobalParams.Num_Groupcores.U - io.rocc.cmd.bits.rs1(15, 12)
  //     tempID := VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){i => io.rocc.cmd.bits.rs2(4 * (i + 5) - 1, 4 * (i + 4))})
  //   }
  //   val nexttempID = RegNext(RegNext(tempID, VecInit(Seq.fill(GlobalParams.Num_Groupcores)(15.U(4.W)))))
    
  //   val resp_funct = RegNext(RegNext(RegNext(io.rocc.cmd.bits.inst.funct)))
    
  //   when(io.rocc.resp.valid && io.rocc.resp.bits.opcode === "b0001011".U && io.rocc.resp.bits.funct === 4.U){
  //     for(i <- 0 until GlobalParams.Num_Groupcores){
  //       when(i.U < NumMaster){
  //         MasterID(i) := nexttempID(GlobalParams.Num_Groupcores - i - 1)
  //         SlaveID(i) := 15.U
  //       }.otherwise{
  //         MasterID(i) := 15.U
  //         SlaveID(i) := nexttempID(GlobalParams.Num_Groupcores - i - 1)
  //       }
  //     }
  //     reg_sels := VecInit((VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){i => io.rocc.resp.bits.data(4 * (i + 5) - 1, 4 * (i + 4))})).map(k => VecInit(k.asBools)))
  //     reg_slavesels := VecInit((0 until 4).map {i =>
  //     Reverse(Cat(VecInit(((VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){i => io.rocc.resp.bits.data(4 * (i + 5) - 1, 4 * (i + 4))})).map(k => VecInit(k.asBools)))).map(_(i))))
  //     })
  //   }
  // }
  
  //Hardware to synchronize ID and selection registers
  /* 
  when(io.hartid === 0.U){
    when(io.rocc.cmd.valid && io.rocc.cmd.bits.inst.funct === 4.U){
      NumID := VecInit(Seq.tabulate(2){i => io.rocc.cmd.bits.rs1(4 * (i + 1) - 1, 4 * i)})
      tempID := VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){i => io.rocc.cmd.bits.rs2(4 * (i + 1) - 1, 4 * i)})
  
      NumMaster := io.rocc.cmd.bits.rs1(3, 0)
      NumSlave := GlobalParams.Num_Groupcores.U - io.rocc.cmd.bits.rs1(3, 0)
    }

    val nexttempID = RegNext(RegNext(tempID, VecInit(Seq.fill(GlobalParams.Num_Groupcores)(15.U(4.W)))))
  
    val resp_funct = RegNext(RegNext(RegNext(io.rocc.cmd.bits.inst.funct)))
    when(io.rocc.resp.valid && resp_funct === 4.U){
      for(i <- 0 until GlobalParams.Num_Groupcores){
        when(i.U < NumMaster){
          MasterID(i) := nexttempID(GlobalParams.Num_Groupcores - i - 1)
          SlaveID(i) := 15.U
        }.otherwise{
          MasterID(i) := 15.U
          SlaveID(i) := nexttempID(GlobalParams.Num_Groupcores - i - 1)
        }
      }
      reg_sels := VecInit((VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){i => io.rocc.resp.bits.data(4 * (i + 1) - 1, 4 * i)})).map(k => VecInit(k.asBools)))
      reg_slavesels := VecInit((0 until 4).map {i =>
      Reverse(Cat(VecInit(((VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){i => io.rocc.resp.bits.data(4 * (i + 1) - 1, 4 * i)})).map(k => VecInit(k.asBools)))).map(_(i))))
      })
      io.selectionout := Cat(reg_sels.flatten.reverse)
    }
    io.numMasterout := NumMaster
    io.numSlaveout := NumSlave
    io.MasterIDout := Cat(MasterID)
    io.SlaveIDout := Cat(SlaveID)
  }.elsewhen(io.hartid === 4.U){
    when(io.rocc.cmd.valid && io.rocc.cmd.bits.inst.funct === 4.U){
      NumID := VecInit(Seq.tabulate(2){i => io.rocc.cmd.bits.rs1(4 * (i + 1) - 1, 4 * i)})
      tempID := VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){i => io.rocc.cmd.bits.rs2(4 * (i + 5) - 1, 4 * (i + 4))})

      NumMaster := io.rocc.cmd.bits.rs1(7, 4)
      NumSlave := GlobalParams.Num_Groupcores.U - io.rocc.cmd.bits.rs1(7, 4)
    }
    val nexttempID = RegNext(RegNext(tempID, VecInit(Seq.fill(GlobalParams.Num_Groupcores)(15.U(4.W)))))
  
    val resp_funct = RegNext(RegNext(RegNext(io.rocc.cmd.bits.inst.funct)))
    when(io.rocc.resp.valid && resp_funct === 4.U){
      for(i <- 0 until GlobalParams.Num_Groupcores){
        when(i.U < NumMaster){
          MasterID(i) := nexttempID(GlobalParams.Num_Groupcores - i - 1)
          SlaveID(i) := 15.U
        }.otherwise{
          MasterID(i) := 15.U
          SlaveID(i) := nexttempID(GlobalParams.Num_Groupcores - i - 1)
        }
      }
      reg_sels := VecInit((VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){i => io.rocc.resp.bits.data(4 * (i + 5) - 1, 4 * (i + 4))})).map(k => VecInit(k.asBools)))
      reg_slavesels := VecInit((0 until 4).map {i =>
      Reverse(Cat((VecInit((VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){i => io.rocc.resp.bits.data(4 * (i + 5) - 1, 4 * (i + 4))})).map(k => VecInit(k.asBools)))).map(_(i))))
      })
      io.selectionout := Cat(reg_sels.flatten.reverse)
    }
    io.numMasterout := NumMaster
    io.numSlaveout := NumSlave
    io.MasterIDout := Cat(MasterID)
    io.SlaveIDout := Cat(SlaveID)
  }.otherwise{
    NumMaster := io.numMasterin
    NumSlave := io.numSlavein
    MasterID := VecInit(Seq.tabulate(GlobalParams.Num_Groupcores)(i => io.MasterIDin(4 * (i + 1) - 1, 4 * i)))
    SlaveID := VecInit(Seq.tabulate(GlobalParams.Num_Groupcores)(i => io.SlaveIDin(4 * (i + 1) - 1, 4 * i)))
    reg_sels := VecInit((VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){
                          i => io.selectionin(4 * (i + 1) - 1, 4 * i)})).map(k => VecInit(k.asBools)))
    reg_slavesels := VecInit((0 until 4).map {i =>
    Reverse(Cat(VecInit(((VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){
                          i => io.selectionin(4 * (i + 1) - 1, 4 * i)})).map(k => VecInit(k.asBools)))).map(_(i))))})
  }
  */
  //FIFO connect

when(isMaster){
  when(Mcheck_call){
    FIFO.io.in.bits  := "h_aaaa".U
    FIFO.io.in.valid := true.B
    start_check      := true.B
  }.elsewhen(Mchecke_call){
    FIFO.io.in.bits  := Cat(mcore_checking.asUInt, 0.U(175.W), instcoun.io.instnum, "h_eeee".U)
    FIFO.io.in.valid := true.B
    //mcore_checking   := false.B
    start_check      := false.B
    end_check        := true.B
  }.elsewhen((!my_log_unit.io.fifo_valid && task_monitor.master_need_endcheck) && mcore_checking && !rf_sentvalid && !mcore_check_done_common){
    FIFO.io.in.bits  := Cat(Mux(task_monitor.user_ecall, instcoun.io.instnum - 1.U, Mux(wb_valid, instcoun.io.instnum + 1.U, instcoun.io.instnum)), "h_bbbb".U)
    FIFO.io.in.valid := true.B
    //mcore_checking   := false.B
    // start_check      := false.B
    // end_check        := true.B
  }.elsewhen((RegNext(my_log_unit.io.fifo_valid) && RegNext(task_monitor.master_need_endcheck)) && mcore_checking && !rf_sentvalid && !mcore_check_done_common){
    FIFO.io.in.bits  := Cat(Mux(task_monitor.user_ecall, instcoun.io.instnum - 1.U, Mux(wb_valid, instcoun.io.instnum + 1.U, instcoun.io.instnum)), "h_bbbb".U)
    FIFO.io.in.valid := true.B
  }.elsewhen(start_check){
    when(isa.io.sent_done && user_mode && !mcore_checking){
      mcore_checking      := true.B
    }.elsewhen(isa.io.sent_done && mcore_checking){
      mcore_checking := false.B
    }.elsewhen(isa.io.sent_done && !user_mode && !mcore_checking){
      sentdone_butuncheck := true.B
    }.elsewhen(!mcore_checking && sentdone_butuncheck && user_mode){
      mcore_checking      := true.B
      sentdone_butuncheck := false.B
    }.elsewhen(mcore_checking){
      sentdone_butuncheck := false.B
    }
    when(rf_sentvalid){
      FIFO.io.in.bits  := isa.io.sent_output
      FIFO.io.in.valid := rf_sentvalid
    }.elsewhen(ls_sentvalid && mcore_checking && user_mode){
      FIFO.io.in.bits  := ls_data
      FIFO.io.in.valid := ls_sentvalid
    }.otherwise{
      FIFO.io.in.bits  := 0.U
      FIFO.io.in.valid := false.B
    }
  }.elsewhen(end_check){
    when(isa.io.sent_done && user_mode && !mcore_checking){
      mcore_checking      := true.B
    }.elsewhen(isa.io.sent_done && mcore_checking){
      mcore_checking := false.B
    }.elsewhen(isa.io.sent_done && !user_mode && !mcore_checking){
      sentdone_butuncheck := true.B
    }.elsewhen(!mcore_checking && sentdone_butuncheck && user_mode){
      mcore_checking      := true.B
      sentdone_butuncheck := false.B
    }.elsewhen(mcore_checking){
      sentdone_butuncheck := false.B
    }
    when(rf_sentvalid){
      FIFO.io.in.bits  := isa.io.sent_output
      FIFO.io.in.valid := rf_sentvalid
    }.otherwise{
      FIFO.io.in.bits  := 0.U
      FIFO.io.in.valid := false.B
    }
    when(isa.io.sent_done){
      end_check     := false.B
    }
  }.otherwise{
    FIFO.io.in.bits  := 0.U
    FIFO.io.in.valid := false.B
  }

  // when(FIFO.io.count <= 2.U){
  //   FIFO.io.out.ready  := false.B
  //   otmmux.io.in.bits  := 0.U
  //   otmmux.io.in.valid := false.B
  // }
  FIFO.io.out.ready  := otmmux.io.in.ready
  otmmux.io.in.bits  := FIFO.io.out.bits
  otmmux.io.in.valid := FIFO.io.out.valid
  
  //FIFO.io.out         <> otmmux.io.in
  for(i <- 0 until GlobalParams.Num_Groupcores){
    io.custom_FIFOout(i).bits  := otmmux.io.out(i).bits
    io.custom_FIFOout(i).valid := otmmux.io.out(i).valid
    otmmux.io.out(i).ready     := io.custom_FIFOout(i).ready
  }
  //io.custom_FIFOout   <> otmmux.io.out
  otmmux.io.sels      := reg_sels(io.hartid)
  custom_reg          := custom_reg + io.hartid + 1.U
  mtomux.io.out.ready := false.B
  mtomux.io.busy_in   := true.B
  mtomux.io.umode_in  := true.B
  for(i <- 0 until GlobalParams.Num_Groupcores){
    mtomux.io.in(i).bits := 0.U
    mtomux.io.in(i).valid := false.B
    mtomux.io.sels(i) := false.B
  }
  mcore_free         := otmmux.io.free_out
  otmmux.io.busy_in  := io.score_busy_in
  otmmux.io.umode_in := io.score_umode_in
  slave_is_umode     := otmmux.io.umode_out.reduce(_ && _)

  instcoun.io.next_check    := isa.io.sent_done

  when(mem_reg_valid && mem_ctrl.mem && mem_ctrl.mem_cmd === "b00111".U && instcoun.io.check_done && isMaster && (check_busy || mcore_checking)){
    sc_undone := true.B
  }.elsewhen(wb_valid && wb_ctrl.mem && wb_ctrl.mem_cmd === "b00111".U){
    sc_undone := false.B
  }
  when(sc_undone && io.dmem.resp.valid){
    sc_cond := io.dmem.resp.bits.data
  }.elsewhen(!sc_undone){
    sc_cond := 1.U
  }
}.otherwise{
  //io.custom_FIFOin <> mtomux.io.in
  //mtomux.io.out <> FIFO.io.in
  FIFO.io.in.bits     := mtomux.io.out.bits
  FIFO.io.in.valid    := mtomux.io.out.valid
  mtomux.io.out.ready := FIFO.io.in.ready
  for(i <- 0 until GlobalParams.Num_Groupcores){
    io.custom_FIFOin(i).ready := mtomux.io.in(i).ready
    mtomux.io.in(i).bits      := io.custom_FIFOin(i).bits
    mtomux.io.in(i).valid     := io.custom_FIFOin(i).valid
  }
  mtomux.io.sels := reg_slavesels(io.hartid).asBools
  
  rdata       := Mux(FIFO.io.empty, 0.U, FIFO.io.out.bits)
  //mode toggle
  mode_signe  := rdata(15, 0) === "h_eeee".U
  mode_signs  := rdata === "h_aaaa".U
  mode_signc  := rdata(15, 0) === "h_bbbb".U

  val checking = Mux(mode_signe, rdata(255) === 1.U, false.B)

  past_sign   := rdata(255,251) === "b10110".U 
  //pc_sign     := rdata(255, 253) === "b111".U
  //rf data
  rf_sign     := rdata(255, 253) === "b101".U
  rf_datatype := rdata(252, 251)
  widx        := rdata(250, 246)
  wrf_data    := rdata(63, 0)
  wfprf_data  := rdata(127, 64)
  
  //ls data
  ls_sign  := rdata(255, 224) === "h_dead_beef".U
  ls_inst  := rdata(223, 192)
  ls_addr  := rdata(191, 128)
  ld_value := rdata(127, 64)  
  st_value := rdata(63, 0)
  
  when(mode_signs){
    start_check               := true.B // for slave core
    check_mode                := true.B
    CP_start                  := true.B
  }.elsewhen(rece_rf_done){
    CP_start   := false.B
  }
  when(mode_signe && checking && (!ls_sign && !rf_sign)){
    score_check_left := rdata(79, 16)
    CP_end           := true.B
  }.elsewhen(mode_signe && !checking && (!ls_sign && !rf_sign)){
    Scheck_end       := true.B
  }.elsewhen(mode_signc && user_mode && (!ls_sign && !rf_sign)){
    inst_left        := rdata(79, 16)
    score_check_left := rdata(79, 16)
    CP_mid_ofchange  := true.B
  }
  when(instcoun.io.score_check_complete){
    check_mode                := false.B
    start_check               := false.B // for slave core
    score_check_done          := true.B
    end_check                 := true.B
  }
  //score_check_done := ((instcoun.io.instnum >= score_check_left) && CP_end) || instcoun.io.check_done
  FIFO.io.out.ready := (mode_signe && !ls_sign && !rf_sign) || (mode_signs && !ls_sign && !rf_sign) || past_sign || (mode_signc && user_mode && !ls_sign && !rf_sign) ||
                       (slave_rece_en && rf_sign && !check_busy && (instcoun.io.instnum === 0.U) && !(CP_mid_ofchange || CP_end)) || 
                       (slave_rece_en && rf_sign && (check_busy || CP_mid_ofchange || CP_end) && score_check_done && !wb_valid && !sbo(32) && !fp_sbo(32) && !io.fpu.fpu_inflight && div.io.req.ready && statesignal) || //(slave_rece_en && rf_sign && CP_end && !wb_valid) ||     //receive rf data
                       (check_busy && check_req_ready && (!(replay_wb || killm_common || mem_xcpt || fpu_kill_mem || score_check_overtaking)) && mem_reg_valid && (mem_ctrl.mem || (mem_ctrl.csr =/= CSR.N && mem_ctrl.wxd)))  //check_resp_valid //receive rf_data or ls_data when need
  
  rf_ready := rf_sign && !FIFO.io.out.ready
  receiving_rf := (rf_count >= 1.U) && (rf_count < 32.U) || (rf_sign && rf_datatype === 1.U && widx === 0.U && FIFO.io.out.ready)
  //rf_read_addr := Counter(FIFO.io.out.ready && rf_sign && rf_datatype === 1.U || rf_datatype === 0.U, 32)._1
  //rf_read_addr := Counter((0 until 32), FIFO.io.out.ready && rf_sign && (rf_datatype === 1.U || rf_datatype === 0.U), rece_rf_done)._1
  rf_read_addr   := Mux(receiving_rf, Mux(rf_sign, rf_count + 1.U, rf_count), 1.U)//Mux(rf_sign, rf_count + 1.U, rf_count)
  //score_comp_done := rece_rf_done && check_busy
  when(task_monitor.priv_change_tonotuser && start_check && (!score_check_done)){
    check_busy                := false.B
    instcoun.io.next_check    := false.B
  }.elsewhen(task_monitor.priv_change_touser && start_check && (instcoun.io.instnum > 0.U || have_jumped || have_reve) && (!score_comp_done) && (ppn_trace_original === ppn_trace)){
    check_busy                := true.B
    instcoun.io.next_check    := false.B
  }.elsewhen(rece_rf_done && !check_busy && user_mode){
    check_busy                := true.B
    instcoun.io.next_check    := true.B
  }.elsewhen(rece_rf_done && (check_busy || CP_mid_ofchange || CP_end)){
    check_busy                := false.B
    score_comp_done           := true.B
    instcoun.io.next_check    := true.B
    end_check                 := false.B
  }.otherwise{
    instcoun.io.next_check    := false.B
  }
  //fp_apply_en := rf_sign && rf_datatype === 2.U && receiving_rf
  //csr_apply_en := rf_sign && rf_datatype === 0.U && receiving_rf
  rece_rf_done := RegNext(rf_wrap)//RegNext(receiving_rf) && !receiving_rf && !rf_ready
  when(rece_rf_done && CP_end){
    CP_end := false.B
  }
  when(rece_rf_done && CP_mid_ofchange){
    CP_mid_ofchange := false.B
  }
  //receive rf s
  when(!check_busy && !score_check_done && !(CP_mid_ofchange || CP_end)){
    when(rf_sign){
      when(rf_datatype === 0.U){
        jump_pc := rdata(47, 8)
        fcsr_out := rdata(7, 0)
        csr_apply_en := FIFO.io.out.ready
      }.elsewhen(rf_datatype === 1.U && widx === 0.U){
          fprf_data    := wfprf_data
          fprf_idx     := widx
          fp_apply_en  := true.B
        //rf.write(widx, wrf_data)
      }.elsewhen(rf_datatype === 1.U && widx > 0.U){
          rf_apply_data := wrf_data
          rf_apply_en   := true.B
          rf_apply_addr := widx
          fprf_data     := wfprf_data
          fprf_idx      := widx
          fp_apply_en   := true.B
      }
    }
  }.elsewhen(check_busy){
    when(rf_sign){
      when(rf_datatype === 1.U){
        when((wrf_data =/= rf_read_data) && (widx =/= 0.U) && (rf_read_addr =/= 0.U)){
          check_rfcp := false.B
        }
        when(wfprf_data =/= io.fpu.frf(widx)){
          check_fprfcp := false.B
        }
      }
    }
  }
  //receive rf e
  //receive ls s
  when(ls_sign && check_req_ready){
    check_req_data_ld    := ld_value
    check_req_data_st    := st_value
    check_req_addr       := ls_addr
  }

  mtomux.io.umode_in := user_mode
  mtomux.io.busy_in  := check_busy || (instcoun.io.instnum > 0.U)
  io.score_umode_out := mtomux.io.umode_out
  io.score_busy_out  := mtomux.io.busy_out // score send busy singal
  otmmux.io.in.valid := false.B
  otmmux.io.in.bits := 0.U
  for(i <- 0 until GlobalParams.Num_Groupcores){
    otmmux.io.out(i).ready := false.B
    otmmux.io.sels(i)      := false.B
    otmmux.io.busy_in(i)   := false.B
    otmmux.io.umode_in(i)  := false.B
  }
}
// .otherwise{
//   when(isMaster){
//     when(Mcheck_call){
//       FIFO.io.in.bits  := "h_aaaa".U
//       FIFO.io.in.valid := true.B
//       start_check      := true.B
//     }.elsewhen(Mchecke_call || (csr.io.trace(0).exception && mcore_checking)){
//       FIFO.io.in.bits  := Cat(instcoun.io.instnum, "h_eeee".U)
//       FIFO.io.in.valid := true.B
//       mcore_checking   := false.B
//       start_check      := false.B
//       end_check        := true.B
//     }.elsewhen(start_check){
//       when(rf_sentvalid){
//         when(mcore_check_free){
//           mcore_checking := true.B
//         }.otherwise{
//           mcore_checking := false.B
//         }
//         FIFO.io.in.bits  := isa.io.sent_output
//         FIFO.io.in.valid := rf_sentvalid
//       }.elsewhen(ls_sentvalid && mcore_checking){
//         FIFO.io.in.bits  := ls_data
//         FIFO.io.in.valid := ls_sentvalid
//       }.otherwise{
//         FIFO.io.in.bits  := 0.U
//         FIFO.io.in.valid := false.B
//       }
//     }.elsewhen(end_check){
//       when(rf_sentvalid){
//         FIFO.io.in.bits  := isa.io.sent_output
//         FIFO.io.in.valid := rf_sentvalid
//       }.otherwise{
//         FIFO.io.in.bits  := 0.U
//         FIFO.io.in.valid := false.B
//       }
//       when(isa.io.sent_done){
//         end_check := false.B
//       }
//     }.otherwise{
//       FIFO.io.in.bits  := 0.U
//       FIFO.io.in.valid := false.B
//     }

//     FIFO.io.out <> otmmux.io.in
//     io.custom_FIFOout <> otmmux.io.out
//     otmmux.io.sels := reg_sels(io.hartid - 4.U)
//     custom_reg := custom_reg + io.hartid + 2.U

//     mtomux.io.out.ready := false.B
//     for(i <- 0 until GlobalParams.Num_Groupcores){
//       mtomux.io.in(i).bits := 0.U
//       mtomux.io.in(i).valid := false.B
//       mtomux.io.sels(i) := false.B
//     }

//     mtomux.io.busy_in := false.B

//     mcore_free := otmmux.io.free_out
//     otmmux.io.busy_in := io.score_busy_in

//     instcoun.io.next_check    := isa.io.sent_done
//   }.otherwise{
//       io.custom_FIFOin <> mtomux.io.in
//       mtomux.io.out <> FIFO.io.in
//       mtomux.io.sels := reg_slavesels(io.hartid - 4.U).asBools
//       val rdata = Mux(FIFO.io.empty, 0.U, FIFO.io.out.bits)
//       //mode toggle
//       val mode_signe = rdata(15, 0) === "h_eeee".U
//       val mode_signs = rdata === "h_aaaa".U
//       //rf data
//       rf_sign         := rdata(255, 253) === "b101".U

//       val rf_datatype = rdata(252, 251)
//       val widx        = rdata(250, 246)
//       val wrf_data    = rdata(63, 0)
//       val wfprf_data  = rdata(127, 64)
//       dontTouch(rf_sign)
//       dontTouch(rf_datatype)
//       dontTouch(widx)
//       dontTouch(wrf_data)
//       dontTouch(wfprf_data)
//       //ls data
//       val ls_sign = rdata(255, 224) === "h_dead_beef".U
//       val ls_inst = rdata(223, 192)
//       val ls_addr = rdata(191, 128)
//       val ld_value = rdata(127, 64)  
//       val st_value = rdata(63, 0)
//       dontTouch(ls_sign)
//       dontTouch(ls_inst)
//       dontTouch(ls_addr)
//       dontTouch(ld_value)
//       dontTouch(st_value)
//       dontTouch(mode_signs)
//       dontTouch(mode_signe)
  
//       when(mode_signs){
//         start_check               := true.B // for slave core
//         check_mode                := true.B
//         CP_start                  := true.B
//       }.elsewhen(rece_rf_done){
//         CP_start   := false.B
//       }
  
//       when(mode_signe){
//         score_check_left := rdata(79, 16)
//         CP_end           := true.B
//       }
//       when(instcoun.io.score_check_complete){
//         check_mode                := false.B
//         start_check               := false.B // for slave core
//         score_check_done          := true.B
//         end_check                 := true.B
//       }
//       //score_check_done := ((instcoun.io.instnum >= score_check_left) && CP_end) || instcoun.io.check_done
  
//       FIFO.io.out.ready := mode_signe || mode_signs ||
//                            (slave_rece_en && rf_sign && !check_busy) || 
//                            (slave_rece_en && rf_sign && check_busy && score_check_done && !wb_valid && !sbo(32) && !fp_sbo(32) && !io.fpu.fpu_inflight && div.io.req.ready && statesignal && !csr.io.interrupt) || //(slave_rece_en && rf_sign && CP_end && !wb_valid) ||     //receive rf data
//                            (check_req_ready && mem_reg_valid && (mem_ctrl.mem || (mem_ctrl.csr =/= CSR.N && mem_ctrl.wxd)))  //check_resp_valid //receive rf_data or ls_data when need
     
//       rf_ready := rf_sign && !FIFO.io.out.ready
//       receiving_rf := rf_sign && FIFO.io.out.ready
//       //rf_read_addr := Counter(FIFO.io.out.ready && rf_sign && rf_datatype === 1.U || rf_datatype === 0.U, 32)._1
//       rf_read_addr := Counter((0 until 32), FIFO.io.out.ready && rf_sign && (rf_datatype === 1.U || rf_datatype === 0.U), rece_rf_done)._1
  
//       //score_comp_done := rece_rf_done && check_busy
//       when(rece_rf_done && !check_busy){
//         check_busy                := true.B
//         instcoun.io.next_check    := true.B
//       }.elsewhen(rece_rf_done && check_busy){
//         check_busy                := false.B
//         score_comp_done           := true.B
//         instcoun.io.next_check    := true.B
//         end_check                 := false.B
//       }.otherwise{
//         instcoun.io.next_check    := false.B
//       }
//       //fp_apply_en := rf_sign && rf_datatype === 2.U && receiving_rf
//       csr_apply_en := rf_sign && rf_datatype === 0.U && receiving_rf
  
//       rece_rf_done := RegNext(receiving_rf) && !receiving_rf && !rf_ready
  
//       when(rece_rf_done && CP_end){
//         CP_end := false.B
//       }
//       //receive rf s
//       when(!check_busy){
//         when(rf_sign){
//           when(rf_datatype === 0.U){
//             jump_pc := rdata(47, 8)
//             fcsr_out := rdata(7, 0)
//           }.elsewhen(rf_datatype === 1.U){
//               rf_apply_data := wrf_data
//               rf_apply_en   := true.B
//               rf_apply_addr := widx
//               fprf_data    := wfprf_data
//               fprf_idx     := widx
//               fp_apply_en  := true.B
//             //rf.write(widx, wrf_data)
//           }
//         }
//       }.elsewhen(check_busy){
//         when(rf_sign){
//           when(rf_datatype === 1.U){
//             when(wrf_data =/= rf_read_data){
//               check_rfcp := false.B
//             }
//             when(wfprf_data =/= io.fpu.frf(widx)){
//               check_fprfcp := false.B
//             }
//           }
//         }
//       }
//       //receive rf e
  
//       //receive ls s
//       when(ls_sign && check_req_ready){
//         check_req_data_ld    := ld_value
//         check_req_data_st    := st_value
//         check_req_addr       := ls_addr
//       }
    
//       mtomux.io.busy_in := check_busy
//       io.score_busy_out := mtomux.io.busy_out // score send busy singal
  
//       otmmux.io.in.valid := false.B
//       otmmux.io.in.bits := 0.U
//       for(i <- 0 until GlobalParams.Num_Groupcores){
//         otmmux.io.out(i).ready := false.B
//         otmmux.io.sels(i) := false.B
//         otmmux.io.busy_in(i) := false.B
//       }
//     } 
//   }


  /*
  for(i <- 0 until GlobalParams.Num_Groupcores){
    when(i.U < NumSlave1){
      assert(!(reg_sels1(SlaveID1(i)).reduce(_ || _)))
      assert(PopCount(reg_slavesels1(SlaveID1(i))) <= 1.U)
    }
    when(i.U < NumSlave2){
      assert(!(reg_sels2(SlaveID2(i) - 4.U).reduce(_ || _)))
      assert(PopCount(reg_slavesels2(SlaveID2(i) - 4.U)) <= 1.U)
    }
    
    when(i.U < NumMaster1){
      assert(PopCount(reg_slavesels1(MasterID1(i))) === 0.U)
    }
    when(i.U < NumMaster2){
      assert(PopCount(reg_slavesels2(MasterID2(i) - 4.U)) === 0.U)
    }
  }
  */


  // gate the clock
  val unpause = csr.io.time(rocketParams.lgPauseCycles-1, 0) === 0.U || csr.io.inhibit_cycle || io.dmem.perf.release || take_pc
  when (unpause) { id_reg_pause := false.B }
  io.cease := csr.io.status.cease && !clock_en_reg
  io.wfi := csr.io.status.wfi
  if (rocketParams.clockGate) {
    long_latency_stall := csr.io.csr_stall || io.dmem.perf.blocked || id_reg_pause && !unpause
    clock_en := clock_en_reg || ex_pc_valid || (!long_latency_stall && io.imem.resp.valid)
    clock_en_reg :=
      ex_pc_valid || mem_pc_valid || wb_pc_valid || // instruction in flight
      io.ptw.customCSRs.disableCoreClockGate || // chicken bit
      !div.io.req.ready || // mul/div in flight
      usingFPU.B && !io.fpu.fcsr_rdy || // long-latency FPU in flight
      io.dmem.replay_next || // long-latency load replaying
      (!long_latency_stall && (ibuf.io.inst(0).valid || io.imem.resp.valid)) // instruction pending

    assert(!(ex_pc_valid || mem_pc_valid || wb_pc_valid) || clock_en)
  }

  // evaluate performance counters
  val icache_blocked = !(io.imem.resp.valid || RegNext(io.imem.resp.valid))
  csr.io.counters foreach { c => c.inc := RegNext(perfEvents.evaluate(c.eventSel)) }

  val coreMonitorBundle = Wire(new CoreMonitorBundle(xLen, fLen))
  dontTouch(coreMonitorBundle)

  coreMonitorBundle.clock := clock
  coreMonitorBundle.reset := reset
  coreMonitorBundle.hartid := io.hartid
  coreMonitorBundle.timer := csr.io.time(31,0)
  coreMonitorBundle.valid := csr.io.trace(0).valid && !csr.io.trace(0).exception
  coreMonitorBundle.pc := csr.io.trace(0).iaddr(vaddrBitsExtended-1, 0).sextTo(xLen)
  coreMonitorBundle.wrenx := wb_wen && !wb_set_sboard
  coreMonitorBundle.wrenf := false.B
  coreMonitorBundle.wrdst := wb_waddr
  coreMonitorBundle.wrdata := rf_wdata
  coreMonitorBundle.rd0src := wb_reg_inst(19,15)
  coreMonitorBundle.rd0val := RegNext(RegNext(ex_rs(0)))
  coreMonitorBundle.rd1src := wb_reg_inst(24,20)
  coreMonitorBundle.rd1val := RegNext(RegNext(ex_rs(1)))
  coreMonitorBundle.inst := csr.io.trace(0).insn
  coreMonitorBundle.excpt := csr.io.trace(0).exception
  coreMonitorBundle.priv_mode := csr.io.trace(0).priv

  if (enableCommitLog) {
    val t = csr.io.trace(0)
    val rd = wb_waddr
    val wfd = wb_ctrl.wfd
    val wxd = wb_ctrl.wxd
    val has_data = wb_wen && !wb_set_sboard

    when (t.valid && !t.exception) {
      when (wfd) {
        printf ("%d 0x%x (0x%x) f%d p%d 0xXXXXXXXXXXXXXXXX\n", t.priv, t.iaddr, t.insn, rd, rd+32.U)
      }
      .elsewhen (wxd && rd =/= 0.U && has_data) {
        printf ("%d 0x%x (0x%x) x%d 0x%x\n", t.priv, t.iaddr, t.insn, rd, rf_wdata)
      }
      .elsewhen (wxd && rd =/= 0.U && !has_data) {
        printf ("%d 0x%x (0x%x) x%d p%d 0xXXXXXXXXXXXXXXXX\n", t.priv, t.iaddr, t.insn, rd, rd)
      }
      .otherwise {
        printf ("%d 0x%x (0x%x)\n", t.priv, t.iaddr, t.insn)
      }
    }

    when (ll_wen && rf_waddr =/= 0.U) {
      printf ("x%d p%d 0x%x\n", rf_waddr, rf_waddr, rf_wdata)
    }
  }
  else {
    when (csr.io.trace(0).valid) {
      midas.targetutils.SynthesizePrintf(printf("C%d: %d [%d] pc=[%x] W[r%d=%x][%d] R[r%d=%x] R[r%d=%x] inst=[%x] wbxcpt:%d eret:%d evec:%x check_busy:%d\n",
         io.hartid, coreMonitorBundle.timer, coreMonitorBundle.valid,
         coreMonitorBundle.pc,
         Mux(wb_ctrl.wxd || wb_ctrl.wfd, coreMonitorBundle.wrdst, 0.U),
         Mux(coreMonitorBundle.wrenx, coreMonitorBundle.wrdata, 0.U),
         coreMonitorBundle.wrenx,
         Mux(wb_ctrl.rxs1 || wb_ctrl.rfs1, coreMonitorBundle.rd0src, 0.U),
         Mux(wb_ctrl.rxs1 || wb_ctrl.rfs1, coreMonitorBundle.rd0val, 0.U),
         Mux(wb_ctrl.rxs2 || wb_ctrl.rfs2, coreMonitorBundle.rd1src, 0.U),
         Mux(wb_ctrl.rxs2 || wb_ctrl.rfs2, coreMonitorBundle.rd1val, 0.U),
         coreMonitorBundle.inst, wb_xcpt, csr.io.eret, csr.io.evec, check_busy))
    }
  }

  //generate copy signal
  sbo(0) := false.B
  for(i <- 1 until 33){ 
    sbo(i) := sbo(i-1) || sboard.read(i.U) 
  }
  
  statesignal := !ex_reg_valid && !mem_reg_valid && !wb_reg_valid
  //this is for the copyvalid input enable signal 
  en_copyvalid := !sbo(32) && !fp_sbo(32) && !io.fpu.fpu_inflight && div.io.req.ready && statesignal && !isa.io.copy_done && ctrl_stalld && q_copyvalid && !fsign
  dontTouch(en_copyvalid)
  
  when(en_copyvalid){
    isa.io.copy_valid := Mux(user_mode, q_copyvalid, q_copyvalid && mcore_checking)
  }
  .otherwise{
    isa.io.copy_valid := false.B
  }

  // CoreMonitorBundle for late latency writes
  val xrfWriteBundle = Wire(new CoreMonitorBundle(xLen, fLen))

  xrfWriteBundle.clock := clock
  xrfWriteBundle.reset := reset
  xrfWriteBundle.hartid := io.hartid
  xrfWriteBundle.timer := csr.io.time(31,0)
  xrfWriteBundle.valid := false.B
  xrfWriteBundle.pc := 0.U
  xrfWriteBundle.wrdst := rf_waddr
  xrfWriteBundle.wrenx := rf_wen && !(csr.io.trace(0).valid && wb_wen && (wb_waddr === rf_waddr))
  xrfWriteBundle.wrenf := false.B
  xrfWriteBundle.wrdata := rf_wdata
  xrfWriteBundle.rd0src := 0.U
  xrfWriteBundle.rd0val := 0.U
  xrfWriteBundle.rd1src := 0.U
  xrfWriteBundle.rd1val := 0.U
  xrfWriteBundle.inst := 0.U
  xrfWriteBundle.excpt := false.B
  xrfWriteBundle.priv_mode := csr.io.trace(0).priv

  PlusArg.timeout(
    name = "max_core_cycles",
    docstring = "Kill the emulation after INT rdtime cycles. Off if 0."
  )(csr.io.time)

  //MyCustomS

  // log store data: 
  //                int:    data from rs1(0/1) + imm
  //                float:  io.fpu.store_data ([31:0] for float and [63:0] for double)
  val log_rs1 = Mux(wb_ctrl.rfs2, 
                    Mux(wb_ctrl.dp, 
                        io.fpu.store_data, 
                        Cat(0.U, io.fpu.store_data(31,0))), 
                    coreMonitorBundle.rd1val)
  // log_rs1 and cM.rd1val is (0/1); rfs2 is (1/2/3); dp is double-floating-point
  
  // log load data: 
  //                int:    data to rd
  //                float:  data to io.fpu.dmem_resp_data
  val log_rd = Mux(wb_ctrl.wfd, 
                  io.fpu.dmem_resp_data, 
                  Mux(coreMonitorBundle.wrenx, 
                      rf_wdata, 
                      0.U))

  val log_dmem_s1_data = (if (fLen == 0) mem_reg_rs2 else Mux(mem_ctrl.fp, Fill((xLen max fLen) / fLen, io.fpu.store_data), mem_reg_rs2))
  val log_dmem_s2_data = RegNext(log_dmem_s1_data)

  

  my_log_unit.io.wb_valid               := wb_valid
  my_log_unit.io.wb_ctrl                := wb_ctrl
  my_log_unit.io.inst                   := coreMonitorBundle.inst

  // for calculating addr           
  my_log_unit.io.imm                    := ImmGen(wb_ctrl.sel_imm, wb_reg_inst)
  my_log_unit.io.rd0val                 := coreMonitorBundle.rd0val
  my_log_unit.io.dmem_s2_data           := log_dmem_s2_data
  my_log_unit.io.csr_rw_rdata           := csr.io.rw.rdata

  // for data
  my_log_unit.io.dmem_resp_data         := io.dmem.resp.bits.data
  my_log_unit.io.rd1val                 := coreMonitorBundle.rd1val
  my_log_unit.io.wrdata                 := coreMonitorBundle.wrdata
  //my_log_unit.io.rd         := log_rd

  val amo_is_w = coreMonitorBundle.inst(14,12) === "b010".U
  val amo_is_d = coreMonitorBundle.inst(14,12) === "b011".U

  // my_log_unit.io.lhs        := Mux(amo_is_w, io.dmem.log_io.amo_lhs >> 32, Mux(amo_is_w, io.dmem.log_io.amo_lhs >> 32, 0.U))
  // my_log_unit.io.rhs        := Mux(amo_is_w, io.dmem.log_io.amo_rhs >> 32, Mux(amo_is_w, io.dmem.log_io.amo_rhs >> 32, 0.U))
  // my_log_unit.io.out        := Mux(amo_is_w, io.dmem.log_io.amo_out >> 32, Mux(amo_is_w, io.dmem.log_io.amo_out >> 32, 0.U))
  

  ls_sentvalid := my_log_unit.io.fifo_valid
  ls_data := my_log_unit.io.fifo_data


  val log_addr = my_log_unit.io.rd0val + my_log_unit.io.imm.asUInt
  dontTouch(log_addr)
  when((wb_ctrl.mem && wb_valid && isSlave && wb_ctrl.mem_cmd.isOneOf(M_XRD, M_XWR)) && check_busy){
    when(check_resp_addr =/= log_addr){
      check_addr := false.B
    }.elsewhen(check_resp_addr === log_addr){
      check_addr := true.B
    }
  }.elsewhen((wb_ctrl.mem && wb_valid && isSlave && (isAMO(wb_ctrl.mem_cmd) || wb_ctrl.mem_cmd.isOneOf(M_XLR, M_XSC))) && check_busy){
    when(check_resp_addr =/= coreMonitorBundle.rd0val){
      check_addr := false.B
    }.elsewhen(check_resp_addr === coreMonitorBundle.rd0val){
      check_addr := true.B
    }
  }

  when((wb_ctrl.mem_cmd === M_XWR || wb_ctrl.mem_cmd === M_XSC || isAMO(wb_ctrl.mem_cmd)) && wb_ctrl.mem && wb_valid && check_busy) {
    //more := io.dmem_s2_data
    when(check_resp_data_st =/= log_dmem_s2_data){
      check_data := false.B
    }.elsewhen(check_resp_data_st === log_dmem_s2_data){
      check_data := true.B
    }
  }

  
  midas.targetutils.SynthesizePrintf(printf("C%d: " +
        "ctrl_killd:%d ctrl_stalld:%d replay_wb:%d replay_wb_common:%d q_copy:%d en_copy:%d ISScopy:%d " +
        "m_checking:%d check_busy:%d instnum:%x check_done:%d slave_is_umode:%d uncheck:%d rfw:%d|%d %x|%x %x|%x timer:%x\n",
         io.hartid, 
         ctrl_killd.asUInt, ctrl_stalld.asUInt, replay_wb.asUInt, replay_wb_common.asUInt, start_copyvalid, en_copyvalid, isa.io.copy_valid,
         mcore_checking.asUInt, check_busy.asUInt, instcoun.io.instnum, instcoun.io.check_done, slave_is_umode, sentdone_butuncheck,
         wb_wen, ll_wen, wb_waddr, ll_waddr, rf_wdata, io.dmem.resp.bits.data(xLen-1, 0), timer_counter
         ))

  midas.targetutils.SynthesizePrintf(printf("C%d: " +
        "rf_widx:%x rf_wdata:%x fprf_wdata:%x rf_ridx:%x rf_rdata:%x fprf_rdata:%x sc_undo:%d %x ppn_count:%x %x\n",
        io.hartid,  
        widx, wrf_data, wfprf_data, rf_read_addr, rf_read_data, io.fpu.frf(widx), sc_undone, sc_cond, ppn_change_count, ppn_trace_original))

  midas.targetutils.SynthesizePrintf(printf("C%d: rf_sv:%d" +
    "FIFO_send_valid:%d FIFO_send_ready:%d FIFO_rece_ready:%d FIFO_rece_valid:%d " +
    "FIFO_count:%x widx:%x ridx:%x FIFO_input:%x FIFO_output:%x\n",
    io.hartid, rf_sentvalid,
    FIFO.io.in.valid.asUInt, FIFO.io.in.ready.asUInt, FIFO.io.out.ready.asUInt, FIFO.io.out.valid.asUInt, 
    FIFO.io.count, FIFO.io.widx, FIFO.io.ridx, FIFO.io.in.bits, FIFO.io.out.bits
  ))

  midas.targetutils.SynthesizePrintf(printf("C%d: " +
    "ex_reg_valid:%d ex_reg_pc:%x ex_reg_inst:%x " +
    "mem_reg_valid:%d mem_reg_pc:%x mem_reg_inst:%x " +
    "wb_valid:%d wb_reg_valid:%d wb_reg_pc:%x wb_reg_inst:%x " +
    "npc:%x pc_reg:%x imem_req_pc:%x %d %x\n",
    io.hartid,
    ex_reg_valid, ex_reg_pc, ex_trace_inst, 
    mem_reg_valid, mem_reg_pc, mem_trace_inst, 
    wb_valid, wb_reg_valid, wb_reg_pc, wb_trace_inst, 
    wb_npc, jump_pc, imem_req_pc, io.imem.req.valid, io.imem.req.bits.pc
  ))

  midas.targetutils.SynthesizePrintf(printf("C%d:" +
    "intrp:%d intrp_ca:%x tra_intrp:%d tra_cause:%x, tra_excep:%d " +
    "tra_priv:%x tra_ppn:%x priv_cha:%d ppn_cha:%d " +
    "monitor_end:%d " +
    "mul|div:%x|%x sbo:%x%x mdone:%d%d\n",
    io.hartid, 
    csr.io.interrupt.asUInt, csr.io.interrupt_cause, csr.io.trace(0).interrupt.asUInt, task_monitor.xcpt_cause, task_monitor.trace_exception.asUInt, 
    task_monitor.trace_priv, task_monitor.trace_ppn, task_monitor.priv_change, task_monitor.ppn_change, 
    task_monitor.master_need_endcheck,
    wb_ctrl.mul, wb_ctrl.div, sbo(32), fp_sbo(32), mcore_check_done, mcore_check_done_common
  ))

  when(isSlave){
    midas.targetutils.SynthesizePrintf(printf("C%d: check_mode:%d rf_count:%x rf_redone:%d receiving_rf:%d rf_sign:%d " +
      "retrf:%d jump:%d reve:%d " +
      "check_req_ready:%d check_resp_valid:%d check_replay:%d%d%d%d %x" +
      "check_addr:%d check_data:%d check_rfcp:%d check_fprfcp:%d check_cp:%d " +
      "return:%d|%d check_done:%d comp_done:%d return_pc:%x " +
      "\n", 
    io.hartid, check_mode, rf_count, rece_rf_done, receiving_rf.asUInt, rf_sign.asUInt, 
    score_return_rf, have_jumped, have_reve, 
    check_req_ready.asUInt, check_resp_valid.asUInt, check_resp_replay, check_replay_flag, check_replay_ready, check_replay_valid, check_replay_inst,
    check_addr.asUInt, check_data.asUInt, check_rfcp.asUInt, check_fprfcp.asUInt, check_cp.asUInt,
    score_return, csr_apply_en, score_check_done, score_comp_done, score_return_pc))
  }
  // when(isSlave){
  //   midas.targetutils.SynthesizePrintf(printf("C%d: check_mode:%d rf_count:%x rf_redone:%d receiving_rf:%d rf_sign:%d " +
  //     "retrf:%d jump:%d reve:%d " +
  //     "check_req_ready:%d check_resp_valid:%d " +
  //     "check_addr:%d check_data:%d check_rfcp:%d check_fprfcp:%d check_cp:%d " +
  //     "return:%d check_done:%d comp_done:%d return_pc:%x " +
  //     "\n", 
  //   io.hartid, check_mode, rf_count, rece_rf_done, receiving_rf.asUInt, rf_sign.asUInt, 
  //   score_return_rf, have_jumped, have_reve, 
  //   check_req_ready.asUInt, check_resp_valid.asUInt, 
  //   check_addr.asUInt, check_data.asUInt, check_rfcp.asUInt, check_fprfcp.asUInt, check_cp.asUInt,
  //   score_return, score_check_done, score_comp_done, score_return_pc))
  // }


  
  
  //MyCustomE

  } // leaving gated-clock domain
  val rocketImpl = withClock (gated_clock) { new RocketImpl }

  def checkExceptions(x: Seq[(Bool, UInt)]) =
    (x.map(_._1).reduce(_||_), PriorityMux(x))

  def coverExceptions(exceptionValid: Bool, cause: UInt, labelPrefix: String, coverCausesLabels: Seq[(Int, String)]): Unit = {
    for ((coverCause, label) <- coverCausesLabels) {
      property.cover(exceptionValid && (cause === coverCause.U), s"${labelPrefix}_${label}")
    }
  }

  def checkHazards(targets: Seq[(Bool, UInt)], cond: UInt => Bool) =
    targets.map(h => h._1 && cond(h._2)).reduce(_||_)

  def encodeVirtualAddress(a0: UInt, ea: UInt) = if (vaddrBitsExtended == vaddrBits) ea else {
    // efficient means to compress 64-bit VA into vaddrBits+1 bits
    // (VA is bad if VA(vaddrBits) != VA(vaddrBits-1))
    val b = vaddrBitsExtended-1
    val a = (a0 >> b).asSInt
    val msb = Mux(a === 0.S || a === -1.S, ea(b), !ea(b-1))
    Cat(msb, ea(b-1, 0))
  }

  class Scoreboard(n: Int, zero: Boolean = false)
  {
    def set(en: Bool, addr: UInt): Unit = update(en, _next | mask(en, addr))
    def clear(en: Bool, addr: UInt): Unit = update(en, _next & ~mask(en, addr))
    def read(addr: UInt): Bool = r(addr)
    def readBypassed(addr: UInt): Bool = _next(addr)

    private val _r = RegInit(0.U(n.W))
    private val r = if (zero) (_r >> 1 << 1) else _r
    private var _next = r
    private var ens = false.B
    private def mask(en: Bool, addr: UInt) = Mux(en, 1.U << addr, 0.U)
    private def update(en: Bool, update: UInt) = {
      _next = update
      ens = ens || en
      when (ens) { _r := _next }
    }
  }
}

class RegFile(n: Int, w: Int, zero: Boolean = false) {
  val rf = Mem(n, UInt(w.W))
  private def access(addr: UInt) = rf(~addr(log2Up(n)-1,0))
  private val reads = ArrayBuffer[(UInt,UInt)]()
  private var canRead = true
  def read(addr: UInt) = {
    require(canRead)
    reads += addr -> Wire(UInt())
    reads.last._2 := Mux(zero.B && addr === 0.U, 0.U, access(addr))
    reads.last._2
  }
  def write(addr: UInt, data: UInt) = {
    canRead = false
    when (addr =/= 0.U) {
      access(addr) := data
      for ((raddr, rdata) <- reads)
        when (addr === raddr) { rdata := data }
    }
  }
}

object ImmGen {
  def apply(sel: UInt, inst: UInt) = {
    val sign = Mux(sel === IMM_Z, 0.S, inst(31).asSInt)
    val b30_20 = Mux(sel === IMM_U, inst(30,20).asSInt, sign)
    val b19_12 = Mux(sel =/= IMM_U && sel =/= IMM_UJ, sign, inst(19,12).asSInt)
    val b11 = Mux(sel === IMM_U || sel === IMM_Z, 0.S,
              Mux(sel === IMM_UJ, inst(20).asSInt,
              Mux(sel === IMM_SB, inst(7).asSInt, sign)))
    val b10_5 = Mux(sel === IMM_U || sel === IMM_Z, 0.U, inst(30,25))
    val b4_1 = Mux(sel === IMM_U, 0.U,
               Mux(sel === IMM_S || sel === IMM_SB, inst(11,8),
               Mux(sel === IMM_Z, inst(19,16), inst(24,21))))
    val b0 = Mux(sel === IMM_S, inst(7),
             Mux(sel === IMM_I, inst(20),
             Mux(sel === IMM_Z, inst(15), 0.U)))

    Cat(sign, b30_20, b19_12, b11, b10_5, b4_1, b0).asSInt
  }
}

class master_flush extends Bundle{
  val check_done   = Bool()
  val complete     = Bool()
}

class slave_flush extends Bundle{
  val overtakeing  = Bool()
  val complete     = Bool()
  val custom_jalr  = Bool()
  val check_done   = Bool()
}

class wb_flush_pipe extends Bundle{
  val m_flush = new master_flush()
  val s_flush = new slave_flush()
  def need_flush: Bool = m_flush.check_done || m_flush.complete || s_flush.complete || s_flush.custom_jalr || s_flush.overtakeing || s_flush.check_done
  def need_replay_pc: Bool = m_flush.check_done || m_flush.complete || s_flush.overtakeing
  def need_jump: Bool = s_flush.custom_jalr
  def need_take_pc: Bool = need_jump || need_replay_pc
}

class need_copy extends Bundle{
  // master core copy signals
  val Mcheck_call         = Bool()
  val Mchecke_call        = Bool()
  val common_endCP        = Bool()
  val common_startCP      = Bool()
  val task_change_endCP   = Bool()
  val task_change_startCP = Bool()

  // slave core copy signal
  val Srecode_call        = Bool()

  def copy: Bool     = Mcheck_call || Mchecke_call || common_endCP || common_startCP || task_change_endCP || task_change_startCP || Srecode_call
}

class task_monitor(implicit p: Parameters) extends CoreBundle()(p){
  val trace_exception             = Bool()
  val xcpt_cause                  = UInt(xLen.W)
  val trace_priv                  = UInt(PRV.SZ.W)
  val trace_slave_priv_umode      = Bool()
  val trace_ppn                   = UInt((maxPAddrBits - pgIdxBits).W)
  val trace_priv_next             = UInt(PRV.SZ.W)
  val trace_slave_priv_umode_next = Bool()
  val trace_ppn_next              = UInt((maxPAddrBits - pgIdxBits).W)

  def priv_change: Bool                 = trace_priv =/= trace_priv_next
  def slave_priv_change: Bool           = trace_slave_priv_umode =/= trace_slave_priv_umode_next
  def ppn_change : Bool                 = trace_ppn  =/= trace_ppn_next
  def priv_change_tonotuser: Bool       = priv_change && (trace_priv_next === PRV.U.U)
  def slave_priv_change_tonotuser: Bool = slave_priv_change && (trace_slave_priv_umode_next === true.B)
  def priv_change_touser: Bool          = priv_change && (trace_priv === PRV.U.U)
  def slave_priv_change_touser: Bool    = priv_change && (trace_slave_priv_umode === true.B)

  def check_notchange: Bool             = !priv_change && !ppn_change
  def task_change:Bool            = ppn_change && (trace_priv === PRV.U.U)

  def user_ecall                  = trace_exception && (xcpt_cause === Causes.user_ecall.U)
  def master_need_endcheck: Bool  = (!check_notchange && priv_change_tonotuser) || (slave_priv_change_tonotuser)
  def master_can_startcheck: Bool = !check_notchange && priv_change_touser
}
