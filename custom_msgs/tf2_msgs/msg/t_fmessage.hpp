#ifndef _TF2_MSGS_MSG_TFMESSAGE_H
#define _TF2_MSGS_MSG_TFMESSAGE_H

#include <iostream>
#include <string>
#include <vector>
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std;

namespace tf2_msgs
{
namespace msg
{
class TFMessage
{
public:
  uint32_t cntPub = 0;
  uint32_t cntSub = 0;

    
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  

  uint32_t copyToBuf(uint8_t *addrPtr)
  {
    uint32_t tmpPub = 0;
    uint32_t arraySize;
    uint32_t stringSize;
    
    if (cntPub%4 >0){
      for(int i=0; i<(4-(cntPub%4)) ; i++){
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 4-(cntPub%4);
    }
    arraySize = transforms.size();
    memcpy(addrPtr,&arraySize,4);
    addrPtr += 4;
    cntPub += 4;

    for(int i=0; i<arraySize ; i++){
      tmpPub = transforms.at(i).copyToBuf(addrPtr, cntPub);
      cntPub += tmpPub;
      addrPtr += tmpPub;
    }
    

    
    
    

    return cntPub;
  }

  uint32_t copyFromBuf(const uint8_t *addrPtr) {
    uint32_t tmpSub = 0;
    uint32_t arraySize;
    uint32_t stringSize;

    
    
    
    {
    if (cntSub%4 >0){
      for(int i=0; i<(4-(cntSub%4)) ; i++){
        addrPtr += 1;
      }
      cntSub += 4-(cntSub%4);
    }
    memcpy(&arraySize,addrPtr,4);
    addrPtr += 4;    
    cntSub += 4;
    for(int i=0;i<arraySize;i++){
      geometry_msgs::msg::TransformStamped buf;
      tmpSub = buf.copyFromBuf(addrPtr);
      transforms.push_back(buf);
      addrPtr += tmpSub;
      cntSub += tmpSub;
    }
    }
    

    
    

    return cntSub;
  }

   void memAlign(uint8_t *addrPtr){
    if (cntPub%4 > 0){
      addrPtr += cntPub;
      for(int i=0; i<(4-(cntPub%4)) ; i++){
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 4-(cntPub%4);
    }
    return;
  }

  uint32_t getTotalSize(){
    uint32_t tmpCntPub = cntPub;
    cntPub = 0;
    return tmpCntPub ;
  }

private:
  std::string type_name = "tf2_msgs::msg::dds_::TFMessage";
};
};
}

namespace message_traits
{
template<>
struct TypeName<tf2_msgs::msg::TFMessage*> {
  static const char* value()
  {
    return "tf2_msgs::msg::dds_::TFMessage_";
  }
};
}

#endif