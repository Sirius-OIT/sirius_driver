#ifndef _GEOMETRY_MSGS_MSG_TRANSFORMSTAMPED_H
#define _GEOMETRY_MSGS_MSG_TRANSFORMSTAMPED_H

#include <iostream>
#include <string>
#include "geometry_msgs/msg/transform.hpp"

using namespace std;

namespace geometry_msgs
{
namespace msg
{
class TransformStamped
{
public:
  uint32_t cntPub = 0;
  uint32_t cntSub = 0;

    
  int32_t sec; uint32_t nanosec; string frame_id;
    
  string child_frame_id
;
    
  geometry_msgs::msg::Transform transform;
  

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

    memcpy(addrPtr,&sec,4);
    addrPtr += 4;
    cntPub += 4;

    memcpy(addrPtr,&nanosec,4);
    addrPtr += 4;
    cntPub += 4;

    stringSize = frame_id.size();
    memcpy(addrPtr,&stringSize,4);
    addrPtr += 4;
    cntPub += 4;
    memcpy(addrPtr,frame_id.c_str(),stringSize);
    addrPtr += stringSize;
    cntPub += stringSize;
  
    
    
    
    if (cntPub%4 >0){
      for(int i=0; i<(4-(cntPub%4)) ; i++){
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 4-(cntPub%4);
    }
    stringSize = child_frame_id
.size();
    memcpy(addrPtr,&stringSize,4);
    addrPtr += 4;
    cntPub += 4;
    memcpy(addrPtr,child_frame_id
.c_str(),stringSize);
    addrPtr += stringSize;
    cntPub += stringSize;

    
    
    
    
    tmpPub = transform.copyToBuf(addrPtr);
    cntPub += tmpPub;
    addrPtr += tmpPub;
    
    
    

    return cntPub;
  }


  uint32_t copyToBuf(uint8_t *addrPtr, uint32_t initial_count_pub)
  {
    uint32_t tmpPub = 0;
    uint32_t arraySize;
    uint32_t stringSize;
    cntPub = initial_count_pub;
    memcpy(addrPtr, &sec, 4);
    addrPtr += 4;
    cntPub += 4;

    memcpy(addrPtr, &nanosec, 4);
    addrPtr += 4;
    cntPub += 4;

    stringSize = frame_id.size();
    memcpy(addrPtr, &stringSize, 4);
    addrPtr += 4;
    cntPub += 4;
    memcpy(addrPtr, frame_id.c_str(), stringSize);
    addrPtr += stringSize;
    cntPub += stringSize;

    if (cntPub % 4 > 0)
    {
      for (int i = 0; i < (4 - (cntPub % 4)); i++)
      {
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 4 - (cntPub % 4);
    }

    stringSize = child_frame_id.size();
    memcpy(addrPtr, &stringSize, 4);
    addrPtr += 4;
    cntPub += 4;
    memcpy(addrPtr, child_frame_id.c_str(), stringSize);
    addrPtr += stringSize;
    cntPub += stringSize;

    if (cntPub % 8 > 0)
    {
      for (int i = 0; i < (8 - (cntPub % 8)); i++)
      {
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 8 - (cntPub % 8);
    }

    tmpPub = transform.copyToBuf(addrPtr);
    cntPub += tmpPub;
    addrPtr += tmpPub;

    return cntPub;
  }

  uint32_t copyFromBuf(const uint8_t *addrPtr) {
    uint32_t tmpSub = 0;
    uint32_t arraySize;
    uint32_t stringSize;

    
    
    if (cntSub%4 >0){
      for(int i=0; i<(4-(cntSub%4)) ; i++){
        addrPtr += 1;
      }
      cntSub += 4-(cntSub%4);
    }
    memcpy(&sec,addrPtr,4);
    addrPtr += 4;
    cntSub += 4;

    memcpy(&nanosec,addrPtr,4);
    addrPtr += 4;
    cntSub += 4;

    memcpy(&stringSize, addrPtr, 4);
    addrPtr += 4;
    cntSub += 4;
    frame_id.resize(stringSize);
    memcpy(&frame_id[0],addrPtr,stringSize);
    addrPtr += stringSize;
    cntSub += stringSize;

    
    
    
    if (cntSub%4 >0){
      for(int i=0; i<(4-(cntSub%4)) ; i++){
        addrPtr += 1;
      }
      cntSub += 4-(cntSub%4);
    }
    memcpy(&stringSize, addrPtr, 4);
    addrPtr += 4;
    cntSub += 4;
    child_frame_id
.resize(stringSize);
    memcpy(&child_frame_id
[0],addrPtr,stringSize);
    addrPtr += stringSize;
    cntSub += stringSize;

    
    
    
    
    tmpSub = transform.copyFromBuf(addrPtr);
    cntSub += tmpSub;
    addrPtr += tmpSub;
    

    
    

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
  std::string type_name = "geometry_msgs::msg::dds_::TransformStamped";
};
};
}

namespace message_traits
{
template<>
struct TypeName<geometry_msgs::msg::TransformStamped*> {
  static const char* value()
  {
    return "geometry_msgs::msg::dds_::TransformStamped_";
  }
};
}

#endif