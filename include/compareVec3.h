#ifndef _H_VEC3_COMPARE_
#define _H_VEC3_COMPARE_

struct vec3_compare{
  bool operator()(const Eigen::Vector3d Vec1, const Eigen::Vector3d Vec2){
    if(Vec1[0] != Vec2[0]){
      return Vec1[0] < Vec2[0];
    }
    else if(Vec1[1] != Vec2[1]){
      return Vec1[1] < Vec2[1];
    }
    else{
      return Vec1[2] < Vec2[2];
    }
  }
};

#endif