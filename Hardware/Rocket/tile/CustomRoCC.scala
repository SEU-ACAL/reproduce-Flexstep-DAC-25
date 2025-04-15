package freechips.rocketchip.rocket

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tile._
import freechips.rocketchip.util._


//custom Rocc
class CustomAccelerator(opcodes: OpcodeSet)
    (implicit p: Parameters) extends LazyRoCC(opcodes, 0, false) {
  override lazy val module = new CustomAcceleratorModule(this)
}

class CustomAcceleratorModule(outer: CustomAccelerator) extends LazyRoCCModuleImp(outer) 
    with HasCoreParameters {
  val cmd = Queue(io.cmd)
  val funct = cmd.bits.inst.funct

  val dowrite                 = funct === 0.U
  val dochange                = funct === 4.U
  val doMcheck                = funct === 3.U
  val doScheck_cp             = funct === 6.U
  val domcheck_running        = funct === 7.U
  val doscheck_receving       = funct === 8.U
  val doscheck_mode           = funct === 10.U

  val scp_done                = io.score_rece_done
  val mcore_running           = io.mcore_runing
  val score_receingrf         = io.score_recerf
  val check_mode              = io.score_checkmode

  dontTouch(io.score_rece_done)
  dontTouch(io.mcore_runing)


  val Num_Mastercores = cmd.bits.rs1(7, 4)
  val Num_Slavecores  = GlobalParams.Num_Groupcores.U - Num_Mastercores
  val Mulicheck       = (Num_Mastercores < Num_Slavecores)
  val Num_checkcore   = Mux(Mulicheck, cmd.bits.rs1(3, 0), 1.U)
  // val Num_Mastercores1 = cmd.bits.rs1(7, 4)
  // val Num_Slavecores1  = GlobalParams.Num_Groupcores.U - Num_Mastercores1
  // val Mulicheck1       = (Num_Mastercores1 < Num_Slavecores1)
  // val Num_checkcore1   = Mux(Mulicheck1, cmd.bits.rs1(3, 0), 1.U)
  
  // val Num_Mastercores2 = cmd.bits.rs1(15, 12)
  // val Num_Slavecores2  = GlobalParams.Num_Groupcores.U - Num_Mastercores2
  // val Mulicheck2       = (Num_Mastercores2 < Num_Slavecores2)
  // val Num_checkcore2   = Mux(Mulicheck2, cmd.bits.rs1(11, 8), 1.U)

  // val HartId1 = VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){ i => cmd.bits.rs2(4 * (i + 1) - 1, 4 * i) })
  // val HartId2 = VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){ i => cmd.bits.rs2(4 * (i + 5) - 1, 4 * (i + 4)) })
  // val sels = 0.U(xLen.W)
  // val sels1 = VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){ i => sels(4 * (i + 1) - 1, 4 * i)})
  // val sels2 = VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){ i => sels(4 * (i + 5) - 1, 4 * (i + 4))})
  // val finalsels1 = WireInit(VecInit(sels1.map{ elem =>
  //   VecInit((0 until elem.getWidth).map(i => elem(i).asUInt))
  // }))
  // val finalsels2 = WireInit(VecInit(sels2.map{ elem =>
  //   VecInit((0 until elem.getWidth).map(i => elem(i).asUInt))
  // }))
  val HartId = VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){ i => cmd.bits.rs2(4 * (i + 1) - 1, 4 * i) })
  val sels_temp = 0.U(xLen.W)
  val sels = VecInit(Seq.tabulate(GlobalParams.Num_Groupcores){ i => sels_temp(4 * (i + 1) - 1, 4 * i)})
  val finalsels = WireInit(VecInit(sels.map{ elem =>
    VecInit((0 until elem.getWidth).map(i => elem(i).asUInt))
  }))

  val rd_val = WireInit(0.U(xLen.W))

  when(dochange){
    for(i <- 0 until GlobalParams.Num_Groupcores){
      when(i.U < Num_Mastercores){
        when(i.U < Num_Slavecores){
          for(j <- 0 until GlobalParams.Num_Groupcores){
            when(j.U === HartId(i)){
              finalsels(HartId(GlobalParams.Num_Groupcores - i - 1))(j) := 1.U
            }.otherwise{
              finalsels(HartId(GlobalParams.Num_Groupcores - i - 1))(j) := 0.U
            }
          }
        }.otherwise{
          for(j <- 0 until GlobalParams.Num_Groupcores){
            finalsels(HartId(GlobalParams.Num_Groupcores - i - 1))(j) := 0.U
          }                                                                          
        }
      }.elsewhen(i.U < Num_Slavecores){
        when(i.U < Num_checkcore){
          finalsels(HartId(GlobalParams.Num_Groupcores.U - Num_Mastercores))(HartId(i)) := 1.U
        }.otherwise{
          finalsels(HartId(GlobalParams.Num_Groupcores.U - Num_Mastercores))(HartId(i)) := 0.U
        }
        for(j <- 0 until GlobalParams.Num_Groupcores){
          finalsels(HartId(GlobalParams.Num_Groupcores - i - 1))(j) := 0.U
        } 
      }.otherwise{
        for(j <- 0 until GlobalParams.Num_Groupcores){
          finalsels(HartId(GlobalParams.Num_Groupcores - i - 1))(j) := 0.U
        }  
      }
      
    }
  }
  // when(dochange){
  //   for(i <- 0 until GlobalParams.Num_Groupcores){
  //     when(i.U < Num_Mastercores1){
  //       when(i.U < Num_Slavecores1){
  //         for(j <- 0 until GlobalParams.Num_Groupcores){
  //           when(j.U === HartId1(i)){
  //             finalsels1(HartId1(GlobalParams.Num_Groupcores - i - 1))(j) := 1.U
  //           }.otherwise{
  //             finalsels1(HartId1(GlobalParams.Num_Groupcores - i - 1))(j) := 0.U
  //           }
  //         }
  //       }.otherwise{
  //         for(j <- 0 until GlobalParams.Num_Groupcores){
  //           finalsels1(HartId1(GlobalParams.Num_Groupcores - i - 1))(j) := 0.U
  //         }                                                                          
  //       }
  //     }.elsewhen(i.U < Num_Slavecores1){
  //       when(i.U < Num_checkcore1){
  //         finalsels1(HartId1(GlobalParams.Num_Groupcores.U - Num_Mastercores1))(HartId1(i)) := 1.U
  //       }.otherwise{
  //         finalsels1(HartId1(GlobalParams.Num_Groupcores.U - Num_Mastercores1))(HartId1(i)) := 0.U
  //       }
  //       for(j <- 0 until GlobalParams.Num_Groupcores){
  //         finalsels1(HartId1(GlobalParams.Num_Groupcores - i - 1))(j) := 0.U
  //       } 
  //     }.otherwise{
  //       for(j <- 0 until GlobalParams.Num_Groupcores){
  //         finalsels1(HartId1(GlobalParams.Num_Groupcores - i - 1))(j) := 0.U
  //       }  
  //     }
      

  //     when(i.U < Num_Mastercores2){
  //       when(i.U < Num_Slavecores2){
  //         for(j <- 0 until GlobalParams.Num_Groupcores){
  //           when(j.U === HartId2(i) - 4.U){
  //             finalsels2(HartId2(GlobalParams.Num_Groupcores - i - 1) - 4.U)(j) := 1.U
  //           }.otherwise{
  //             finalsels2(HartId2(GlobalParams.Num_Groupcores - i - 1) - 4.U)(j) := 0.U
  //           }
  //         }
  //       }.otherwise{
  //         for(j <- 0 until GlobalParams.Num_Groupcores){
  //           finalsels2(HartId2(GlobalParams.Num_Groupcores - i - 1) - 4.U)(j) := 0.U
  //         }
  //       }
  //     }.elsewhen(i.U < Num_Slavecores2){
  //       when(i.U < Num_checkcore2){
  //         finalsels2(HartId2(GlobalParams.Num_Groupcores.U - Num_Mastercores2) - 4.U)(HartId2(i) - 4.U) := 1.U
  //       }.otherwise{
          
  //       }
  //       finalsels2(HartId2(GlobalParams.Num_Groupcores.U - Num_Mastercores2) - 4.U)(HartId2(i) - 4.U) := 1.U
  //       for(j <- 0 until GlobalParams.Num_Groupcores){
  //           finalsels2(HartId2(GlobalParams.Num_Groupcores - i - 1) - 4.U)(j) := 0.U
  //       } 
  //     }.otherwise{
  //       for(j <- 0 until GlobalParams.Num_Groupcores){
  //           finalsels2(HartId2(GlobalParams.Num_Groupcores - i - 1) - 4.U)(j) := 0.U
  //         }
  //     }
  //   }
  // }
  
  
  // rd_val := MuxCase(0.U,
  //             Seq(
  //               dowrite           -> "b0011".U,
  //               dochange          -> Cat(0.U(16.W), Cat(finalsels2.flatten.reverse), Cat(finalsels1.flatten.reverse)),
  //               doMcheck          -> "b1100".U,
  //               doScheck_cp       -> Cat(scp_done.asUInt, 0.U(3.W)),
  //               domcheck_running  -> Cat(mcore_running.asUInt, 1.U(3.W)),
  //               doscheck_receving -> Cat(score_receingrf.asUInt, 2.U(3.W)),
  //               doscheck_mode     -> Cat(check_mode.asUInt, 3.U(3.W))
  //             )
  // )

  rd_val := MuxCase(0.U,
              Seq(
                dowrite           -> "b0011".U,
                dochange          -> Cat(0.U(48.W), Cat(finalsels.flatten.reverse)),
                doMcheck          -> "b1100".U,
                doScheck_cp       -> Cat(scp_done.asUInt, 0.U(3.W)),
                domcheck_running  -> Cat(mcore_running.asUInt, 1.U(3.W)),
                doscheck_receving -> Cat(score_receingrf.asUInt, 2.U(3.W)),
                doscheck_mode     -> Cat(check_mode.asUInt, 3.U(3.W))
              )
  )
  
    
  val doResp = cmd.bits.inst.xd

  cmd.ready := true.B

  io.resp.valid          := cmd.valid && doResp
  io.resp.bits.rd        := cmd.bits.inst.rd
  io.resp.bits.funct     := funct
  io.resp.bits.opcode    := cmd.bits.inst.opcode
  io.resp.bits.data      := rd_val
    
  io.busy := cmd.valid
  io.interrupt := false.B
  // The parts of the command are as follows
  // inst - the parts of the instruction itself
  //   opcode
  //   rd - destination register number
  //   rs1 - first source register number
  //   rs2 - second source register number
  //   funct
  //   xd - is the destination register being used?
  //   xs1 - is the first source register being used?
  //   xs2 - is the second source register being used?
  // rs1 - the value of source register 1
  // rs2 - the value of source register 2
}
//custom Rocc end

