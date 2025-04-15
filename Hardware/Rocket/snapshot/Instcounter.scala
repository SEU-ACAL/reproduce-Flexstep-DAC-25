package freechips.rocketchip.snapshot

import chisel3._
import chisel3.util._


class Instcounter extends Module{
  val io = IO(new Bundle{
     val isMaster               =    Input(Bool())
     val start                  =    Input(Bool())
     val wb_valid               =    Input(Bool())
     val copy_done              =    Input(Bool())
     val next_check             =    Input(Bool())
     val CP_end                 =    Input(Bool())
     val CP_mid                 =    Input(Bool())
     val Mchecke_call           =    Input(Bool())
     val Schecke_call           =    Input(Bool())
     val user_mode              =    Input(Bool())
     val mcore_endcheck         =    Input(Bool())
     val mode_signc             =    Input(Bool())
     val inst_left_wire         =    Input(UInt(64.W))
     val instnum_threshold      =    Input(UInt(64.W))
     val inst_left              =    Input(UInt(64.W))

     val copy_valid             =    Output(Bool())
     val check_done             =    Output(Bool())
     val check_done_common      =    Output(Bool())
     val score_check_complete   =    Output(Bool())
     val mcore_check_complete   =    Output(Bool())
     val instnum                =    Output(UInt(64.W))    
})
  val instnumber     =    RegInit(0.U(64.W))
   
  io.instnum := instnumber
  //copy request
  when(io.start){
    when(io.wb_valid){
      instnumber := instnumber + 1.U
    }.elsewhen(io.next_check){
      instnumber := 0.U
    }.otherwise{
      instnumber := instnumber
    }
    when(instnumber >= io.instnum_threshold){
      io.copy_valid := true.B
      instnumber := 0.U
      //io.check_done := true.B
    }.otherwise{
      io.copy_valid := false.B
      //io.check_done := false.B
    }


    io.mcore_check_complete := io.Mchecke_call
    io.score_check_complete := Mux(io.wb_valid, ((instnumber >= io.inst_left - 1.U) && io.CP_end && io.user_mode), ((instnumber >= io.inst_left) && io.CP_end && io.user_mode)) || io.Schecke_call
    io.check_done           := Mux(io.isMaster, io.mcore_endcheck || (io.wb_valid && io.user_mode && (instnumber >= io.instnum_threshold - 1.U)), 
                                  Mux(io.wb_valid, (((instnumber >= io.inst_left - 1.U) && io.CP_mid) || (instnumber >= io.instnum_threshold - 1.U) || ((instnumber >= io.inst_left_wire - 1.U) && io.mode_signc)) && io.user_mode, 
                                      (instnumber >= io.inst_left) && io.CP_mid && io.user_mode))
    io.check_done_common    := io.wb_valid && io.user_mode && instnumber >= io.instnum_threshold - 1.U
                            
    when((io.CP_mid || io.CP_end) && (instnumber >= io.inst_left)){
     instnumber := 0.U
    }

    
    
    dontTouch(instnumber)
  }.otherwise{
    io.score_check_complete := Mux(io.CP_end, instnumber >= io.inst_left && io.user_mode, false.B)  
    io.mcore_check_complete := false.B
    when(io.next_check){
       instnumber := 0.U
    }
    io.check_done_common    := false.B
    io.copy_valid := false.B
    io.check_done := Mux(io.CP_mid, instnumber >= io.inst_left && io.user_mode, false.B)  
  }
  
}
