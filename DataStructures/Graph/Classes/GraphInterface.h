/**********************************************************************************

 Copyright (c) 2023-2025 Patrick Steil
 Copyright (c) 2019-2022 KIT ITI Algorithmics Group

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************************/
#pragma once

#include <limits>

#include "../../../Helpers/Meta.h"
#include "../../../Helpers/String/String.h"
#include "../../../Helpers/TaggedInteger.h"
#include "../../../Helpers/Types.h"
#include "../../Attributes/Attributes.h"
#include "../../Geometry/Point.h"

class EdgeWithFromVertex {
 public:
  EdgeWithFromVertex(const Edge edge, const Vertex fromVertex)
      : edge(edge), fromVertex(fromVertex) {}

  inline operator Edge() const noexcept { return edge; }

  inline Vertex operator[](
      const ImplementationDetail::FromVertexType) const noexcept {
    return fromVertex;
  }

  inline Vertex get(const ImplementationDetail::FromVertexType) const noexcept {
    return fromVertex;
  }

 public:
  const Edge edge;
  const Vertex fromVertex;
};

inline std::string cleanGraphType(const std::string& rawType) noexcept {
  if (rawType.empty()) return rawType;
  std::string cleanType =
      String::replaceAll(rawType, Meta::type<Vertex>(), "Vertex");
  cleanType = String::replaceAll(cleanType, Meta::type<Edge>(), "Edge");
  cleanType = String::replaceAll(cleanType, "Meta::", "");
  cleanType = String::replaceAll(cleanType, "> ", ">");
  cleanType = String::replaceAll(cleanType, "Implementation", "");
  for (size_t i = 0; i < NumberOfAttributeNames; i++) {
    std::stringstream oldAttributeType;
    oldAttributeType << "<" << i << "u";
    std::stringstream newAttributeType;
    newAttributeType << "<" << attributeToString(i);
    cleanType = String::replaceAll(cleanType, oldAttributeType.str(),
                                   newAttributeType.str());
  }
  return cleanType;
}

template <typename T>
inline std::string graphType() noexcept {
  return cleanGraphType(Meta::type<Meta::PlainType<T>>());
}

template <typename T>
inline std::string graphType(T&&) noexcept {
  return graphType<T>();
}

template <typename LIST_OF_VERTEX_ATTRIBUTES, typename LIST_OF_EDGE_ATTRIBUTES>
class DynamicGraphImplementation;
template <typename LIST_OF_VERTEX_ATTRIBUTES, typename LIST_OF_EDGE_ATTRIBUTES>
class StaticGraphImplementation;
template <typename LIST_OF_VERTEX_ATTRIBUTES, typename LIST_OF_EDGE_ATTRIBUTES>
class EdgeListImplementation;

namespace Graph {
template <typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM,
          typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO,
          typename... ATTRIBUTE_NAME_CHANGES>
inline void move(
    StaticGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&&
        from,
    StaticGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to,
    const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
template <typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM,
          typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO,
          typename... ATTRIBUTE_NAME_CHANGES>
inline void move(
    StaticGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&&
        from,
    DynamicGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to,
    const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
template <typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM,
          typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO,
          typename... ATTRIBUTE_NAME_CHANGES>
inline void move(
    StaticGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&&
        from,
    EdgeListImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to,
    const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);

template <typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM,
          typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO,
          typename... ATTRIBUTE_NAME_CHANGES>
inline void move(
    DynamicGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&&
        from,
    StaticGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to,
    const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
template <typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM,
          typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO,
          typename... ATTRIBUTE_NAME_CHANGES>
inline void move(
    DynamicGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&&
        from,
    DynamicGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to,
    const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
template <typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM,
          typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO,
          typename... ATTRIBUTE_NAME_CHANGES>
inline void move(
    DynamicGraphImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&&
        from,
    EdgeListImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to,
    const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);

template <typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM,
          typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO,
          typename... ATTRIBUTE_NAME_CHANGES>
inline void move(
    EdgeListImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from,
    StaticGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to,
    const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
template <typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM,
          typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO,
          typename... ATTRIBUTE_NAME_CHANGES>
inline void move(
    EdgeListImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from,
    DynamicGraphImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to,
    const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
template <typename VERTEX_ATTRIBUTES_FROM, typename EDGE_ATTRIBUTES_FROM,
          typename VERTEX_ATTRIBUTES_TO, typename EDGE_ATTRIBUTES_TO,
          typename... ATTRIBUTE_NAME_CHANGES>
inline void move(
    EdgeListImplementation<VERTEX_ATTRIBUTES_FROM, EDGE_ATTRIBUTES_FROM>&& from,
    EdgeListImplementation<VERTEX_ATTRIBUTES_TO, EDGE_ATTRIBUTES_TO>& to,
    const ATTRIBUTE_NAME_CHANGES... attributeNameChanges);
}  // namespace Graph

template <typename LIST_OF_VERTEX_ATTRIBUTES, typename LIST_OF_EDGE_ATTRIBUTES>
using StaticGraph = StaticGraphImplementation<
    Meta::SortAttributes<LIST_OF_VERTEX_ATTRIBUTES>,
    Meta::InsertAttribute<Attribute<ToVertex, Vertex>,
                          Meta::SortAttributes<LIST_OF_EDGE_ATTRIBUTES>>>;

template <typename LIST_OF_VERTEX_ATTRIBUTES, typename LIST_OF_EDGE_ATTRIBUTES>
using DynamicGraph = DynamicGraphImplementation<
    Meta::InsertAttribute<
        Attribute<BeginOut, Edge>,
        Meta::InsertAttribute<
            Attribute<OutDegree, size_t>,
            Meta::InsertAttribute<
                Attribute<IncomingEdges, std::vector<Edge>>,
                Meta::SortAttributes<LIST_OF_VERTEX_ATTRIBUTES>>>>,
    Meta::InsertAttribute<
        Attribute<Valid, bool>,
        Meta::InsertAttribute<
            Attribute<IncomingEdgePointer, size_t>,
            Meta::InsertAttribute<
                Attribute<FromVertex, Vertex>,
                Meta::InsertAttribute<
                    Attribute<ToVertex, Vertex>,
                    Meta::SortAttributes<LIST_OF_EDGE_ATTRIBUTES>>>>>>;

template <typename LIST_OF_VERTEX_ATTRIBUTES, typename LIST_OF_EDGE_ATTRIBUTES>
using EdgeList = EdgeListImplementation<
    Meta::SortAttributes<LIST_OF_VERTEX_ATTRIBUTES>,
    Meta::InsertAttribute<
        Attribute<FromVertex, Vertex>,
        Meta::InsertAttribute<Attribute<ToVertex, Vertex>,
                              Meta::SortAttributes<LIST_OF_EDGE_ATTRIBUTES>>>>;
