/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** @author Jia Pan */

#ifndef FCL_BVH_MODEL_INL_H
#define FCL_BVH_MODEL_INL_H

#include "fcl/geometry/bvh/BVH_model.h"
#include <algorithm>
#include <memory>
#include <stdexcept>

namespace fcl
{

//==============================================================================
template <typename BV>
BVHModelType BVHModel<BV>::getModelType() const
{
  if(num_tris && num_vertices)
    return BVH_MODEL_TRIANGLES;
  else if(num_vertices)
    return BVH_MODEL_POINTCLOUD;
  else
    return BVH_MODEL_UNKNOWN;
}

//==============================================================================
template <typename BV>
BVHModel<BV>::BVHModel()
: vertices(),
  tri_indices(),
  prev_vertices(),
  build_state(BVH_BUILD_STATE_EMPTY),
  bv_splitter(std::make_shared<detail::BVFitter<BV>>(detail::SPLIT_METHOD_MEAN)),
  bv_fitter(std::make_shared<detail::BVFitter<BV>>()),
  primitive_indices()
  bvs()
{
  // Do nothing
}

//==============================================================================
template <typename BV>
BVHModel<BV>::BVHModel(const BVHModel<BV>& other)
  : CollisionGeometry<S>(other),
    vertices(other.vertices),
    tri_indices(other.tri_indices),
    prev_vertices(other.prev_vertices),
    build_state(other.build_state),
    bv_splitter(other.bv_splitter),
    bv_fitter(other.bv_fitter),
    primitive_indices(other.primitive_indices),
    bvs(other.bvs)
{
  // Do nothing
}

//==============================================================================
template <typename BV>
BVHModel<BV>::~BVHModel()
{
  // Do nothing
}

//==============================================================================
template <typename BV>
const BVNode<BV>& BVHModel<BV>::getBV(unsigned int id) const
{
  return bvs[id];
}

//==============================================================================
template <typename BV>
BVNode<BV>& BVHModel<BV>::getBV(unsigned int id)
{
  return bvs[id];
}

//==============================================================================
template <typename BV>
unsigned int BVHModel<BV>::getNumBVs() const
{
  return bvs.size();
}

//==============================================================================
template <typename BV>
OBJECT_TYPE BVHModel<BV>::getObjectType() const
{
  return OT_BVH;
}

//==============================================================================
template <typename BV>
struct GetNodeTypeImpl
{
  static NODE_TYPE run()
  {
    return BV_UNKNOWN;
  }
};

//==============================================================================
template <typename BV>
NODE_TYPE BVHModel<BV>::getNodeType() const
{
  return GetNodeTypeImpl<BV>::run();
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::beginModel(unsigned int num_tris_ = 0, unsigned int num_vertices_ = 0)
{
  if(build_state != BVH_BUILD_STATE_EMPTY)
  {
    vertices.clear();
    tri_indices.clear();
    bvs.clear();
    prev_vertices.clear();
    primitive_indices.clear();

    std::cerr << "BVH Warning! Call beginModel() on a BVHModel that is not empty. This model was cleared and previous triangles/vertices were lost.\n";
    build_state = BVH_BUILD_STATE_EMPTY;
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if(num_tris_ > 0)
  {
    try
    {
      tri_indices.reserve(num_tris_);
    }
    catch (const std::exception& e)
    {
      std::cerr << "BVH Error! Out of memory for tri_indices array on BeginModel() call!\n";
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }
  }

  if(num_vertices_ > 0)
  {
    try
    {
      vertices.reserve(num_vertices_);
    }
    catch(const std::exception& e)
    {
      std::cerr << "BVH Error! Out of memory for vertices array on BeginModel() call!\n";
      return BVH_ERR_MODEL_OUT_OF_MEMORY;
    }
  }
  
  build_state = BVH_BUILD_STATE_BEGUN;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::addVertex(const Vector3<S>& p)
{
  if(build_state != BVH_BUILD_STATE_BEGUN)
  {
    std::cerr << "BVH Warning! Call addVertex() in a wrong order. addVertex() was ignored. Must do a beginModel() to clear the model for addition of new vertices.\n";
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  try
  {
    vertices.push_back(p);
  }
  catch(const std::exception& e)
  {
    std::cerr << "BVH Error! Out of memory for vertices array on addVertex() call!\n";
    return BVH_ERR_MODEL_OUT_OF_MEMORY;
  }

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::addTriangle(const Vector3<S>& p1, const Vector3<S>& p2, const Vector3<S>& p3)
{
  if(build_state == BVH_BUILD_STATE_PROCESSED)
  {
    std::cerr << "BVH Warning! Call addTriangle() in a wrong order. addTriangle() was ignored. Must do a beginModel() to clear the model for addition of new triangles.\n";
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  auto offset = vertices.size();

  try
  {
    vertices.push_back(p1);
    vertices.push_back(p2);
    vertices.push_back(p3);
  }
  catch(const std::exception& e)
  {
    std::cerr << "BVH Error! Out of memory for vertices array on addTriangle() call!\n";
    return BVH_ERR_MODEL_OUT_OF_MEMORY;
  }

  try
  {
    tri_indices.push_back(offset, offset + 1, offset + 2);
  }
  catch(const std::exception& e)
  {
    std::cerr << "BVH Error! Out of memory for tri_indices array on addTriangle() call!\n";
    return BVH_ERR_MODEL_OUT_OF_MEMORY;
  }
  
  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::addSubModel(const std::vector<Vector3<S>>& ps)
{
  if(build_state == BVH_BUILD_STATE_PROCESSED)
  {
    std::cerr << "BVH Warning! Call addSubModel() in a wrong order. addSubModel() was ignored. Must do a beginModel() to clear the model for addition of new vertices.\n";
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  try
  {
    vertices.reserve(vertices.size() + ps.size());
  }
  catch(const std::exception& e)
  {
    std::cerr << "BVH Error! Out of memory for vertices array on addSubModel() call!\n";
    return BVH_ERR_MODEL_OUT_OF_MEMORY;
  }
  
  for(const auto& p : ps)
  {
    vertices.push_back(p);
  }

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::addSubModel(const std::vector<Vector3<S>>& ps, const std::vector<Triangle>& ts)
{
  auto offset = vertices.size();

  auto return_code = this->addSubModel(ps);

  if (return_code != BVH_OK)
  {
    return return_code;
  }

  try
  {
    tri_indices.reserve(tri_indices.size() + ts.size())
  }
  catch(const std::exception& e)
  {
    std::cerr << "BVH Error! Out of memory for tri_indices array on addSubModel() call!\n";
    return BVH_ERR_MODEL_OUT_OF_MEMORY;
  }
  
  for(const auto& t : ts)
  {
    tri_indices.emplace_back(t[0] + offset, t[1] + offset, t[2] + offset);
  }

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::endModel()
{
  if(build_state != BVH_BUILD_STATE_BEGUN)
  {
    std::cerr << "BVH Warning! Call endModel() in wrong order. endModel() was ignored.\n";
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if(tri_indices.size() == 0 && vertices.size() == 0)
  {
    std::cerr << "BVH Error! endModel() called on model with no triangles and vertices.\n";
    return BVH_ERR_BUILD_EMPTY_MODEL;
  }

  try
  {
    tri_indices.shrink_to_fit();
  }
  catch(const std::exception& e)
  {
    std::cerr << "BVH Error! Out of memory for tri_indices array in endModel() call!\n";
    return BVH_ERR_MODEL_OUT_OF_MEMORY;
  }

  try
  {
    vertices.shrink_to_fit();
  }
  catch(const std::exception& e)
  {
    std::cerr << "BVH Error! Out of memory for vertices array in endModel() call!\n";
    return BVH_ERR_MODEL_OUT_OF_MEMORY;
  }

  // construct BVH tree
  auto reverved_size = 2 * std::max(vertices.size(), tri_indices.size()) - 1;

  try
  {
    bvs.reserve(reverved_size);
    primitive_indices.reserve(reverved_size);
  }
  catch(const std::exception& e)
  {
    std::cerr << "BVH Error! Out of memory for BV array in endModel()!\n";
    return BVH_ERR_MODEL_OUT_OF_MEMORY;
  }

  buildTree();

  // finish constructing
  build_state = BVH_BUILD_STATE_PROCESSED;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::beginReplaceModel()
{
  if(build_state != BVH_BUILD_STATE_PROCESSED)
  {
    std::cerr << "BVH Error! Call beginReplaceModel() on a BVHModel that has no previous frame.\n";
    return BVH_ERR_BUILD_EMPTY_PREVIOUS_FRAME;
  }

  prev_vertices.clear();

  num_vertex_updated = 0;

  build_state = BVH_BUILD_STATE_REPLACE_BEGUN;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::replaceVertex(const Vector3<S>& p)
{
  if(build_state != BVH_BUILD_STATE_REPLACE_BEGUN)
  {
    std::cerr << "BVH Warning! Call replaceVertex() in a wrong order. replaceVertex() was ignored. Must do a beginReplaceModel() for initialization.\n";
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  vertices[num_vertex_updated] = p;
  num_vertex_updated++;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::replaceTriangle(const Vector3<S>& p1, const Vector3<S>& p2, const Vector3<S>& p3)
{
  if(build_state != BVH_BUILD_STATE_REPLACE_BEGUN)
  {
    std::cerr << "BVH Warning! Call replaceTriangle() in a wrong order. replaceTriangle() was ignored. Must do a beginReplaceModel() for initialization.\n";
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  vertices[num_vertex_updated] = p1; num_vertex_updated++;
  vertices[num_vertex_updated] = p2; num_vertex_updated++;
  vertices[num_vertex_updated] = p3; num_vertex_updated++;
  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::replaceSubModel(const std::vector<Vector3<S>>& ps)
{
  if(build_state != BVH_BUILD_STATE_REPLACE_BEGUN)
  {
    std::cerr << "BVH Warning! Call replaceSubModel() in a wrong order. replaceSubModel() was ignored. Must do a beginReplaceModel() for initialization.\n";
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  for(const auto& p : ps)
  {
    vertices[num_vertex_updated] = p;
    num_vertex_updated++;
  }
  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::endReplaceModel(bool refit, bool bottomup)
{
  if(build_state != BVH_BUILD_STATE_REPLACE_BEGUN)
  {
    std::cerr << "BVH Warning! Call endReplaceModel() in a wrong order. endReplaceModel() was ignored. \n";
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if(num_vertex_updated != num_vertices)
  {
    std::cerr << "BVH Error! The replaced model should have the same number of vertices as the old model.\n";
    return BVH_ERR_INCORRECT_DATA;
  }

  if(refit)  // refit, do not change BVH structure
  {
    refitTree(bottomup);
  }
  else // reconstruct bvh tree based on current frame data
  {
    buildTree();
  }

  build_state = BVH_BUILD_STATE_PROCESSED;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::beginUpdateModel()
{
  if(build_state != BVH_BUILD_STATE_PROCESSED && build_state != BVH_BUILD_STATE_UPDATED)
  {
    std::cerr << "BVH Error! Call beginUpdatemodel() on a BVHModel that has no previous frame.\n";
    return BVH_ERR_BUILD_EMPTY_PREVIOUS_FRAME;
  }

  // swap prev_vertives and vertices.
  if(prev_vertices)
  {
    vertices.swap(prev_vertices);
  }
  else
  {
    prev_vertices = std::move(vertices);
    vertices = std::vector<Vector3<S>>(num_vertices);
  }

  num_vertex_updated = 0;

  build_state = BVH_BUILD_STATE_UPDATE_BEGUN;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::updateVertex(const Vector3<S>& p)
{
  if(build_state != BVH_BUILD_STATE_UPDATE_BEGUN)
  {
    std::cerr << "BVH Warning! Call updateVertex() in a wrong order. updateVertex() was ignored. Must do a beginUpdateModel() for initialization.\n";
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  vertices[num_vertex_updated] = p;
  num_vertex_updated++;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::updateTriangle(const Vector3<S>& p1, const Vector3<S>& p2, const Vector3<S>& p3)
{
  if(build_state != BVH_BUILD_STATE_UPDATE_BEGUN)
  {
    std::cerr << "BVH Warning! Call updateTriangle() in a wrong order. updateTriangle() was ignored. Must do a beginUpdateModel() for initialization.\n";
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  vertices[num_vertex_updated] = p1; num_vertex_updated++;
  vertices[num_vertex_updated] = p2; num_vertex_updated++;
  vertices[num_vertex_updated] = p3; num_vertex_updated++;
  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::updateSubModel(const std::vector<Vector3<S>>& ps)
{
  if(build_state != BVH_BUILD_STATE_UPDATE_BEGUN)
  {
    std::cerr << "BVH Warning! Call updateSubModel() in a wrong order. updateSubModel() was ignored. Must do a beginUpdateModel() for initialization.\n";
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  for(const auto& p : ps)
  {
    vertices[num_vertex_updated] = p;
    num_vertex_updated++;
  }
  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::endUpdateModel(bool refit, bool bottomup)
{
  if(build_state != BVH_BUILD_STATE_UPDATE_BEGUN)
  {
    std::cerr << "BVH Warning! Call endUpdateModel() in a wrong order. endUpdateModel() was ignored. \n";
    return BVH_ERR_BUILD_OUT_OF_SEQUENCE;
  }

  if(num_vertex_updated != vertices.size())
  {
    std::cerr << "BVH Error! The updated model should have the same number of vertices as the old model.\n";
    return BVH_ERR_INCORRECT_DATA;
  }

  if(refit)  // refit, do not change BVH structure
  {
    refitTree(bottomup);
  }
  else // reconstruct bvh tree based on current frame data
  {
    buildTree();

    // then refit

    refitTree(bottomup);
  }


  build_state = BVH_BUILD_STATE_UPDATED;

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::memUsage(int msg) const
{
  auto mem_bv_list = sizeof(BV) * bvs.size();
  auto mem_tri_list = sizeof(Triangle) * tri_indices.size();
  auto mem_vertex_list = sizeof(Vector3<S>) * vertices.size();

  auto total_mem = mem_bv_list + mem_tri_list + mem_vertex_list + sizeof(BVHModel<BV>);
  if(msg)
  {
    std::cerr << "Total for model " << total_mem << " bytes.\n";
    std::cerr << "BVs: " << bvs.size() << " allocated.\n";
    std::cerr << "Tris: " << tri_indices.size() << " allocated.\n";
    std::cerr << "Vertices: " << vertices.size() << " allocated.\n";
  }

  return BVH_OK;
}

//==============================================================================
template <typename BV>
void BVHModel<BV>::makeParentRelative()
{
  makeParentRelativeRecurse(
        0, Matrix3<S>::Identity(), Vector3<S>::Zero());
}

//==============================================================================
template <typename BV>
Vector3<typename BV::S> BVHModel<BV>::computeCOM() const
{
  S vol = 0;
  Vector3<S> com = Vector3<S>::Zero();
  for(const auto& tri : tri_indices)
  {
    S d_six_vol = (vertices[tri[0]].cross(vertices[tri[1]])).dot(vertices[tri[2]]);
    vol += d_six_vol;
    com.noalias() += (vertices[tri[0]] + vertices[tri[1]] + vertices[tri[2]]) * d_six_vol;
  }

  return com / (vol * 4);
}

//==============================================================================
template <typename BV>
typename BV::S BVHModel<BV>::computeVolume() const
{
  S vol = 0;
  for(const auto& tri : tri_indices)
  {
    S d_six_vol = (vertices[tri[0]].cross(vertices[tri[1]])).dot(vertices[tri[2]]);
    vol += d_six_vol;
  }

  return vol / 6;
}

//==============================================================================
template <typename BV>
Matrix3<typename BV::S> BVHModel<BV>::computeMomentofInertia() const
{
  Matrix3<S> C = Matrix3<S>::Zero();

  Matrix3<S> C_canonical;
  C_canonical << 1/ 60.0, 1/120.0, 1/120.0,
      1/120.0, 1/ 60.0, 1/120.0,
      1/120.0, 1/120.0, 1/ 60.0;

  for(const auto& tri : tri_indices)
  {
    const Vector3<S>& v1 = vertices[tri[0]];
    const Vector3<S>& v2 = vertices[tri[1]];
    const Vector3<S>& v3 = vertices[tri[2]];
    S d_six_vol = (v1.cross(v2)).dot(v3);
    Matrix3<S> A;
    A.row(0) = v1;
    A.row(1) = v2;
    A.row(2) = v3;
    C.noalias() += A.transpose() * C_canonical * A * d_six_vol;
  }

  S trace_C = C(0, 0) + C(1, 1) + C(2, 2);

  Matrix3<S> m;
  m << trace_C - C(0, 0), -C(0, 1), -C(0, 2),
      -C(1, 0), trace_C - C(1, 1), -C(1, 2),
      -C(2, 0), -C(2, 1), trace_C - C(2, 2);

  return m;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::buildTree()
{
  // set BVFitter
  bv_fitter->set(vertices.data(), tri_indices.data(), getModelType());
  // set SplitRule
  bv_splitter->set(vertices.data(), tri_indices.data(), getModelType());

  num_bvs = 1;

  unsigned int num_primitives = 0;
  switch(getModelType())
  {
  case BVH_MODEL_TRIANGLES:
    num_primitives = tri_indices.size();
    break;
  case BVH_MODEL_POINTCLOUD:
    num_primitives = vertices.size();
    break;
  default:
    std::cerr << "BVH Error: Model type not supported!\n";
    return BVH_ERR_UNSUPPORTED_FUNCTION;
  }

  for(unsigned int i = 0; i < num_primitives; ++i)
    primitive_indices[i] = i;
  recursiveBuildTree(0, 0, num_primitives);

  bv_fitter->clear();
  bv_splitter->clear();

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::recursiveBuildTree(unsigned int bv_id, unsigned int first_primitive, unsigned int num_primitives)
{
  BVHModelType type = getModelType();
  auto bvnode = bvs.emplace(bvs.begin() + bv_id);
  auto cur_primitive_indices = primitive_indices.emplace(primitive_indices.begin() + first_primitive);

  // constructing BV
  BV bv = bv_fitter->fit(&(*cur_primitive_indices), num_primitives);
  bv_splitter->computeRule(bv, &(*cur_primitive_indices), num_primitives);

  bvnode->bv = bv;
  bvnode->first_primitive = first_primitive;
  bvnode->num_primitives = num_primitives;

  if(num_primitives == 1)
  {
    bvnode->first_child = -((*cur_primitive_indices) + 1);
  }
  else
  {
    bvnode->first_child = bvs.size();
    num_bvs += 2;

    unsigned int c1 = 0;
    for(unsigned int i = 0; i < num_primitives; ++i)
    {
      Vector3<S> p;
      if(type == BVH_MODEL_POINTCLOUD) p = vertices[cur_primitive_indices[i]];
      else if(type == BVH_MODEL_TRIANGLES)
      {
        const Triangle& t = tri_indices[cur_primitive_indices[i]];
        const Vector3<S>& p1 = vertices[t[0]];
        const Vector3<S>& p2 = vertices[t[1]];
        const Vector3<S>& p3 = vertices[t[2]];
        p.noalias() = (p1 + p2 + p3) / 3.0;
      }
      else
      {
        std::cerr << "BVH Error: Model type not supported!\n";
        return BVH_ERR_UNSUPPORTED_FUNCTION;
      }


      // loop invariant: up to (but not including) index c1 in group 1,
      // then up to (but not including) index i in group 2
      //
      //  [1] [1] [1] [1] [2] [2] [2] [x] [x] ... [x]
      //                   c1          i
      //
      if(bv_splitter->apply(p)) // in the right side
      {
        // do nothing
      }
      else
      {
        std::swap(cur_primitive_indices[i], cur_primitive_indices[c1]);
        c1++;
      }
    }


    if((c1 == 0) || (c1 == num_primitives)) c1 = num_primitives / 2;

    auto num_first_half = c1;

    recursiveBuildTree(bvnode->leftChild(), first_primitive, num_first_half);
    recursiveBuildTree(bvnode->rightChild(), first_primitive + num_first_half, num_primitives - num_first_half);
  }

  return BVH_OK;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::refitTree(bool bottomup)
{
  if(bottomup)
    return refitTree_bottomup();
  else
    return refitTree_topdown();
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::refitTree_bottomup()
{
  int res = recursiveRefitTree_bottomup(0);

  return res;
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::recursiveRefitTree_bottomup(int bv_id)
{
  BVNode<BV>* bvnode = bvs + bv_id;
  if(bvnode->isLeaf())
  {
    BVHModelType type = getModelType();
    int primitive_id = -(bvnode->first_child + 1);
    if(type == BVH_MODEL_POINTCLOUD)
    {
      BV bv;

      if(prev_vertices)
      {
        Vector3<S> v[2];
        v[0] = prev_vertices[primitive_id];
        v[1] = vertices[primitive_id];
        fit(v, 2, bv);
      }
      else
        fit(vertices + primitive_id, 1, bv);

      bvnode->bv = bv;
    }
    else if(type == BVH_MODEL_TRIANGLES)
    {
      BV bv;
      const Triangle& triangle = tri_indices[primitive_id];

      if(prev_vertices)
      {
        Vector3<S> v[6];
        for(int i = 0; i < 3; ++i)
        {
          v[i] = prev_vertices[triangle[i]];
          v[i + 3] = vertices[triangle[i]];
        }

        fit(v, 6, bv);
      }
      else
      {
        Vector3<S> v[3];
        for(int i = 0; i < 3; ++i)
        {
          v[i] = vertices[triangle[i]];
        }

        fit(v, 3, bv);
      }

      bvnode->bv = bv;
    }
    else
    {
      std::cerr << "BVH Error: Model type not supported!\n";
      return BVH_ERR_UNSUPPORTED_FUNCTION;
    }
  }
  else
  {
    recursiveRefitTree_bottomup(bvnode->leftChild());
    recursiveRefitTree_bottomup(bvnode->rightChild());
    bvnode->bv = bvs[bvnode->leftChild()].bv + bvs[bvnode->rightChild()].bv;
  }

  return BVH_OK;
}

//==============================================================================
template <typename S, typename BV>
struct MakeParentRelativeRecurseImpl
{
  static void run(BVHModel<BV>& model,
                  int bv_id,
                  const Matrix3<S>& parent_axis,
                  const Vector3<S>& parent_c)
  {
    if(!model.bvs[bv_id].isLeaf())
    {
      MakeParentRelativeRecurseImpl<S, BV> tmp1;
      tmp1(model, model.bvs[bv_id].first_child, parent_axis, model.bvs[bv_id].getCenter());

      MakeParentRelativeRecurseImpl<S, BV> tmp2;
      tmp2(model, model.bvs[bv_id].first_child + 1, parent_axis, model.bvs[bv_id].getCenter());
    }

    model.bvs[bv_id].bv = translate(model.bvs[bv_id].bv, -parent_c);
  }
};

//==============================================================================
template <typename BV>
void BVHModel<BV>::makeParentRelativeRecurse(
    int bv_id,
    const Matrix3<S>& parent_axis,
    const Vector3<S>& parent_c)
{
  MakeParentRelativeRecurseImpl<typename BV::S, BV>::run(
        *this, bv_id, parent_axis, parent_c);
}

//==============================================================================
template <typename BV>
int BVHModel<BV>::refitTree_topdown()
{
  bv_fitter->set(vertices, prev_vertices, tri_indices, getModelType());
  for(int i = 0; i < num_bvs; ++i)
  {
    BV bv = bv_fitter->fit(primitive_indices + bvs[i].first_primitive, bvs[i].num_primitives);
    bvs[i].bv = bv;
  }

  bv_fitter->clear();

  return BVH_OK;
}

//==============================================================================
template <typename BV>
void BVHModel<BV>::computeLocalAABB()
{
  AABB<S> aabb_;
  for(int i = 0; i < num_vertices; ++i)
  {
    aabb_ += vertices[i];
  }

  this->aabb_center = aabb_.center();

  this->aabb_radius = 0;
  for(int i = 0; i < num_vertices; ++i)
  {
    S r = (this->aabb_center - vertices[i]).squaredNorm();
    if(r > this->aabb_radius) this->aabb_radius = r;
  }

  this->aabb_radius = sqrt(this->aabb_radius);

  this->aabb_local = aabb_;
}

//==============================================================================
template <typename S>
struct MakeParentRelativeRecurseImpl<S, OBB<S>>
{
  static void run(BVHModel<OBB<S>>& model,
                  int bv_id,
                  const Matrix3<S>& parent_axis,
                  const Vector3<S>& parent_c)
  {
    OBB<S>& obb = model.bvs[bv_id].bv;
    if(!model.bvs[bv_id].isLeaf())
    {
      MakeParentRelativeRecurseImpl<S, OBB<S>> tmp1;
      tmp1(model, model.bvs[bv_id].first_child, obb.axis, obb.To);

      MakeParentRelativeRecurseImpl<S, OBB<S>> tmp2;
      tmp2(model, model.bvs[bv_id].first_child + 1, obb.axis, obb.To);
    }

    // make self parent relative
    obb.axis = parent_axis.transpose() * obb.axis;
    obb.To = (obb.To - parent_c).transpose() * parent_axis;
  }
};

//==============================================================================
template <typename S>
struct MakeParentRelativeRecurseImpl<S, RSS<S>>
{
  static void run(BVHModel<RSS<S>>& model,
                  int bv_id,
                  const Matrix3<S>& parent_axis,
                  const Vector3<S>& parent_c)
  {
    RSS<S>& rss = model.bvs[bv_id].bv;
    if(!model.bvs[bv_id].isLeaf())
    {
      MakeParentRelativeRecurseImpl<S, RSS<S>> tmp1;
      tmp1(model, model.bvs[bv_id].first_child, rss.axis, rss.To);

      MakeParentRelativeRecurseImpl<S, RSS<S>> tmp2;
      tmp2(model, model.bvs[bv_id].first_child + 1, rss.axis, rss.To);
    }

    // make self parent relative
    rss.axis = parent_axis.transpose() * rss.axis;
    rss.To = (rss.To - parent_c).transpose() * parent_axis;
  }
};

//==============================================================================
template <typename S>
struct MakeParentRelativeRecurseImpl<S, OBBRSS<S>>
{
  static void run(BVHModel<OBBRSS<S>>& model,
                  int bv_id,
                  const Matrix3<S>& parent_axis,
                  const Vector3<S>& parent_c)
  {
    OBB<S>& obb = model.bvs[bv_id].bv.obb;
    RSS<S>& rss = model.bvs[bv_id].bv.rss;
    if(!model.bvs[bv_id].isLeaf())
    {
      MakeParentRelativeRecurseImpl<S, RSS<S>> tmp1;
      tmp1(model, model.bvs[bv_id].first_child, obb.axis, obb.To);

      MakeParentRelativeRecurseImpl<S, RSS<S>> tmp2;
      tmp2(model, model.bvs[bv_id].first_child + 1, obb.axis, obb.To);
    }

    // make self parent relative
    obb.axis = parent_axis.transpose() * obb.axis;
    rss.axis = obb.axis;

    obb.To = (obb.To - parent_c).transpose() * parent_axis;
    rss.To = obb.To;
  }
};

//==============================================================================
template <typename S>
struct GetNodeTypeImpl<AABB<S>>
{
  static NODE_TYPE run()
  {
    return BV_AABB;
  }
};

//==============================================================================
template <typename S>
struct GetNodeTypeImpl<OBB<S>>
{
  static NODE_TYPE run()
  {
    return BV_OBB;
  }
};

//==============================================================================
template <typename S>
struct GetNodeTypeImpl<RSS<S>>
{
  static NODE_TYPE run()
  {
    return BV_RSS;
  }
};

//==============================================================================
template <typename S>
struct GetNodeTypeImpl<kIOS<S>>
{
  static NODE_TYPE run()
  {
    return BV_kIOS;
  }
};

//==============================================================================
template <typename S>
struct GetNodeTypeImpl<OBBRSS<S>>
{
  static NODE_TYPE run()
  {
    return BV_OBBRSS;
  }
};

//==============================================================================
template <typename S>
struct GetNodeTypeImpl<KDOP<S, 16>>
{
  static NODE_TYPE run()
  {
    return BV_KDOP16;
  }
};

//==============================================================================
template <typename S>
struct GetNodeTypeImpl<KDOP<S, 18>>
{
  static NODE_TYPE run()
  {
    return BV_KDOP18;
  }
};

//==============================================================================
template <typename S>
struct GetNodeTypeImpl<KDOP<S, 24>>
{
  static NODE_TYPE run()
  {
    return BV_KDOP24;
  }
};

} // namespace fcl

#endif
