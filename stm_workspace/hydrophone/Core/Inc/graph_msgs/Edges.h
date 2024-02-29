#ifndef _ROS_graph_msgs_Edges_h
#define _ROS_graph_msgs_Edges_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace graph_msgs
{

  class Edges : public ros::Msg
  {
    public:
      uint32_t node_ids_length;
      typedef uint32_t _node_ids_type;
      _node_ids_type st_node_ids;
      _node_ids_type * node_ids;
      uint32_t weights_length;
      typedef double _weights_type;
      _weights_type st_weights;
      _weights_type * weights;

    Edges():
      node_ids_length(0), st_node_ids(), node_ids(nullptr),
      weights_length(0), st_weights(), weights(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->node_ids_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->node_ids_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->node_ids_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->node_ids_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->node_ids_length);
      for( uint32_t i = 0; i < node_ids_length; i++){
      *(outbuffer + offset + 0) = (this->node_ids[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->node_ids[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->node_ids[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->node_ids[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->node_ids[i]);
      }
      *(outbuffer + offset + 0) = (this->weights_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->weights_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->weights_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->weights_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->weights_length);
      for( uint32_t i = 0; i < weights_length; i++){
      union {
        double real;
        uint64_t base;
      } u_weightsi;
      u_weightsi.real = this->weights[i];
      *(outbuffer + offset + 0) = (u_weightsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_weightsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_weightsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_weightsi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_weightsi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_weightsi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_weightsi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_weightsi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->weights[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t node_ids_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      node_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      node_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      node_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->node_ids_length);
      if(node_ids_lengthT > node_ids_length)
        this->node_ids = (uint32_t*)realloc(this->node_ids, node_ids_lengthT * sizeof(uint32_t));
      node_ids_length = node_ids_lengthT;
      for( uint32_t i = 0; i < node_ids_length; i++){
      this->st_node_ids =  ((uint32_t) (*(inbuffer + offset)));
      this->st_node_ids |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_node_ids |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_node_ids |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_node_ids);
        memcpy( &(this->node_ids[i]), &(this->st_node_ids), sizeof(uint32_t));
      }
      uint32_t weights_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      weights_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      weights_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      weights_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->weights_length);
      if(weights_lengthT > weights_length)
        this->weights = (double*)realloc(this->weights, weights_lengthT * sizeof(double));
      weights_length = weights_lengthT;
      for( uint32_t i = 0; i < weights_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_weights;
      u_st_weights.base = 0;
      u_st_weights.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_weights.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_weights.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_weights.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_weights.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_weights.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_weights.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_weights.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_weights = u_st_weights.real;
      offset += sizeof(this->st_weights);
        memcpy( &(this->weights[i]), &(this->st_weights), sizeof(double));
      }
     return offset;
    }

    virtual const char * getType() override { return "graph_msgs/Edges"; };
    virtual const char * getMD5() override { return "1dcd54afd0b0c0fbebeb59dbdda4c026"; };

  };

}
#endif
