package freechips.rocketchip.util

import chisel3._

object GlobalParams {
  val Num_cores = 4
  val Num_Groups = 1
  val Num_Groupcores = Num_cores / Num_Groups
  val List_hartid = (0 until Num_cores by 1).toList
  // val List_hartid1 = (0 until Num_Groupcores by 1).toList
  // val List_hartid2 = (Num_Groupcores until Num_cores by 1).toList

  var List_MasterId = List(0, 1)
  var Num_Mastercores = List_MasterId.length
  var Num_Slavecores = Num_Groupcores - Num_Mastercores
  var List_SlaveId = List_hartid.filterNot(List_MasterId.contains)
  // var List_MasterId1 = List(0, 1)
  // var List_MasterId2 = List(4, 5)
  // var Num_Mastercores1 = List_MasterId1.length
  // var Num_Slavecores1 = Num_Groupcores - Num_Mastercores1
  // var Num_Mastercores2 = List_MasterId2.length
  // var Num_Slavecores2 = Num_Groupcores - Num_Mastercores2
  // var List_SlaveId1 = List_hartid1.filterNot(List_MasterId1.contains)
  // var List_SlaveId2 = List_hartid2.filterNot(List_MasterId2.contains)
  
  val Data_width = 256
  val Data_type = UInt(Data_width.W)
  val depth = 32
}
