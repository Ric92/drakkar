//---------------------------------------------------------------------------------------------------------------------
//  DRAKKAR
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Ricardo Lopez Lopez (a.k.a. ric92) ricloplop@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to
//  deal in the Software without restriction, including without limitation the
//  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
//  sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//  IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#ifndef DRAKKAR_ENTITY_H_
#define DRAKKAR_ENTITY_H_

#include <iostream>
#include <fstream>
#include <mutex>

#include <eigen3/Eigen/Core>

namespace drakkar{
    
class Entity
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Entity();

    /// Get current estimated pose
    Eigen::Matrix4f pose() const;

    ~Entity();

private:

    mutable std::mutex dataLock_;

    /// Pose 
    Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();
    
};

}

#endif