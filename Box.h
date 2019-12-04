// Code to make box to pick.
#ifndef TERM_BOX_H
#define TERM_BOX_H

#include "gengine/Transform.h"
#include "gengine/PoseTree.h"
#include "gengine/glmx/pose.h"

class Box {
public:
    // Get this box's model matrix from hand coordinates of the given body.
    static glm::mat4 getTransformFromHands( const glmx::pose& poseState, 
                                            const PoseTree& poseTree, 
                                            const std::string &lhand, 
                                            const std::string &rhand,
                                            const glm::mat4& globalTrans = glm::mat4(1.0f));
};

#endif
