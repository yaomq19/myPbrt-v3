
/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_SHAPE_H
#define PBRT_CORE_SHAPE_H

// core/shape.h*
#include "pbrt.h"
#include "geometry.h"
#include "interaction.h"
#include "memory.h"
#include "transform.h"

namespace pbrt {

// Shape Declarations
class Shape {
  public:
    // Shape Interface
    Shape(const Transform *ObjectToWorld, const Transform *WorldToObject,
          bool reverseOrientation);
    virtual ~Shape();
    //返回该形状在物体空间的包围盒
    virtual Bounds3f ObjectBound() const = 0;
    //返回该形状在世界空间的包围盒（提供默认实现但可以被重写）
    virtual Bounds3f WorldBound() const;
    //确定是否存在相交，并在isect处返回相交细节信息用于后续处理（比如着色）
    //相交测试一般以以下的形式进行最为高效：
    //1、传入在世界空间的光线信息
    //2、由具体的Shape类负责将光线转换到物体坐标空间
    //3、在物理坐标空间执行相交测试计算
    //4、将所得信息转回世界空间
    //note：这么转换两次是因为在物体空间执行相交计算带来的性能提升要高于两次转换的开销
    virtual bool Intersect(const Ray &ray, Float *tHit,
                           SurfaceInteraction *isect,
                           bool testAlphaTexture = true) const = 0;
    //只确定是否存在相交而不计算其所有细节
    virtual bool IntersectP(const Ray &ray,
                            bool testAlphaTexture = true) const {
        return Intersect(ray, nullptr, nullptr, testAlphaTexture);
    }
    //物体空间中形状的表面积
    virtual Float Area() const = 0;
    // Sample a point on the surface of the shape and return the PDF with
    // respect to area on the surface.
    virtual Interaction Sample(const Point2f &u, Float *pdf) const = 0;
    virtual Float Pdf(const Interaction &) const { return 1 / Area(); }

    // Sample a point on the shape given a reference point |ref| and
    // return the PDF with respect to solid angle from |ref|.
    virtual Interaction Sample(const Interaction &ref, const Point2f &u,
                               Float *pdf) const;
    virtual Float Pdf(const Interaction &ref, const Vector3f &wi) const;

    // Returns the solid angle subtended by the shape w.r.t. the reference
    // point p, given in world space. Some shapes compute this value in
    // closed-form, while the default implementation uses Monte Carlo
    // integration; the nSamples parameter determines how many samples are
    // used in this case.
    virtual Float SolidAngle(const Point3f &p, int nSamples = 512) const;

public:
    // Shape Public Data
    const Transform *ObjectToWorld, *WorldToObject;//物体坐标空间到世界坐标空间的互转变换
    const bool reverseOrientation;//表示表面法线指向外部还是内部
    const bool transformSwapsHandedness;//记录ObjectToWorld变换是否翻转左右手性
};

}  // namespace pbrt

#endif  // PBRT_CORE_SHAPE_H
