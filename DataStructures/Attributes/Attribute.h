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

#include <iostream>

#include "../../Helpers/Meta.h"
#include "../../Helpers/String/Enumeration.h"
#include "../../Helpers/String/String.h"
#include "AttributeNames.h"

// Introduction of Attribute, a struct that represents a pair of an
// AttributeName and a Type at compile time
template <AttributeNameType ATTRIBUTE_NAME, typename TYPE>
struct Attribute {
  using Type = TYPE;
  inline constexpr static AttributeNameType Name = ATTRIBUTE_NAME;
  inline constexpr static const char* String = attributeToString(Name);

  inline static std::string toString() noexcept {
    std::stringstream result;
    result << String << " (" << Meta::type<Type>() << ")";
    return result.str();
  }
};

// Some template meta programming functions needed to manipulate sorted Lists
// of Attribute
namespace Meta {

// INSERT ATTRIBUTE
namespace Implementation {
template <typename ATTRIBUTE, typename ATTRIBUTE_LIST,
          typename... RESULTING_LIST>
struct InsertAttribute;

template <typename ATTRIBUTE, typename... RESULTING_LIST>
struct InsertAttribute<ATTRIBUTE, List<>, RESULTING_LIST...>
    : List<RESULTING_LIST..., ATTRIBUTE> {};

template <AttributeNameType NAME, typename TYPE_A, typename TYPE_B,
          typename... ATTRIBUTE_LIST, typename... RESULTING_LIST>
struct InsertAttribute<Attribute<NAME, TYPE_A>,
                       List<Attribute<NAME, TYPE_B>, ATTRIBUTE_LIST...>,
                       RESULTING_LIST...>
    : List<RESULTING_LIST..., Attribute<NAME, TYPE_A>, ATTRIBUTE_LIST...> {};

template <AttributeNameType NAME_A, typename TYPE_A, AttributeNameType NAME_B,
          typename TYPE_B, typename... ATTRIBUTE_LIST,
          typename... RESULTING_LIST>
struct InsertAttribute<Attribute<NAME_A, TYPE_A>,
                       List<Attribute<NAME_B, TYPE_B>, ATTRIBUTE_LIST...>,
                       RESULTING_LIST...>
    : IF<(NAME_A < NAME_B),
         List<RESULTING_LIST..., Attribute<NAME_A, TYPE_A>,
              Attribute<NAME_B, TYPE_B>, ATTRIBUTE_LIST...>,
         InsertAttribute<Attribute<NAME_A, TYPE_A>, List<ATTRIBUTE_LIST...>,
                         RESULTING_LIST..., Attribute<NAME_B, TYPE_B>>> {};
}  // namespace Implementation

template <typename ATTRIBUTE, typename ATTRIBUTE_LIST>
using InsertAttribute =
    typename Implementation::InsertAttribute<ATTRIBUTE, ATTRIBUTE_LIST>::Type;

// REMOVE ATTRIBUTE
namespace Implementation {
template <AttributeNameType NAME, typename ATTRIBUTE_LIST,
          typename... RESULTING_LIST>
struct RemoveAttribute;

template <AttributeNameType NAME, typename... RESULTING_LIST>
struct RemoveAttribute<NAME, List<>, RESULTING_LIST...>
    : List<RESULTING_LIST...> {};

template <AttributeNameType NAME, typename TYPE, typename... ATTRIBUTE_LIST,
          typename... RESULTING_LIST>
struct RemoveAttribute<NAME, List<Attribute<NAME, TYPE>, ATTRIBUTE_LIST...>,
                       RESULTING_LIST...>
    : RemoveAttribute<NAME, List<ATTRIBUTE_LIST...>, RESULTING_LIST...> {};

template <AttributeNameType NAME_A, AttributeNameType NAME_B, typename TYPE,
          typename... ATTRIBUTE_LIST, typename... RESULTING_LIST>
struct RemoveAttribute<NAME_A, List<Attribute<NAME_B, TYPE>, ATTRIBUTE_LIST...>,
                       RESULTING_LIST...>
    : RemoveAttribute<NAME_A, List<ATTRIBUTE_LIST...>, RESULTING_LIST...,
                      Attribute<NAME_B, TYPE>> {};
}  // namespace Implementation

template <AttributeNameType NAME, typename ATTRIBUTE_LIST>
using RemoveAttribute =
    typename Implementation::RemoveAttribute<NAME, ATTRIBUTE_LIST>::Type;

// SORT ATTRIBUTES (by AttributeName, using insertion sort)
namespace Implementation {
template <typename ATTRIBUTE_LIST>
struct SortAttributes;

template <>
struct SortAttributes<List<>> : List<> {};

template <typename ATTRIBUTE, typename... ATTRIBUTE_LIST>
struct SortAttributes<List<ATTRIBUTE, ATTRIBUTE_LIST...>>
    : InsertAttribute<ATTRIBUTE,
                      typename SortAttributes<List<ATTRIBUTE_LIST...>>::Type> {
};
}  // namespace Implementation

template <typename ATTRIBUTE_LIST>
using SortAttributes =
    typename Implementation::SortAttributes<ATTRIBUTE_LIST>::Type;

// CONTAINS ATTRIBUTE
namespace Implementation {
template <AttributeNameType ATTRIBUTE_NAME, typename ATTRIBUTE_LIST>
struct ContainsAttribute;

template <AttributeNameType ATTRIBUTE_NAME>
struct ContainsAttribute<ATTRIBUTE_NAME, List<>> : False {};

template <AttributeNameType ATTRIBUTE_NAME, AttributeNameType NAME_A,
          typename TYPE_A, typename... ATTRIBUTE_LIST>
struct ContainsAttribute<ATTRIBUTE_NAME,
                         List<Attribute<NAME_A, TYPE_A>, ATTRIBUTE_LIST...>>
    : IF<ATTRIBUTE_NAME != NAME_A,
         ContainsAttribute<ATTRIBUTE_NAME, List<ATTRIBUTE_LIST...>>, True> {};
}  // namespace Implementation

template <AttributeNameType ATTRIBUTE_NAME, typename ATTRIBUTE_LIST>
inline constexpr bool ContainsAttribute() {
  return Implementation::ContainsAttribute<ATTRIBUTE_NAME,
                                           ATTRIBUTE_LIST>::Value;
}

// FIND ATTRIBUTE LIST
namespace Implementation {
template <AttributeNameType ATTRIBUTE_NAME, typename ATTRIBUTE_LIST>
struct FindAttributeList {
  static_assert(ATTRIBUTE_NAME != ATTRIBUTE_NAME,
                "ATTRIBUTE_LIST does not contain an attribute with "
                "ATTRIBUTE_NAME as name!");
};

template <AttributeNameType ATTRIBUTE_NAME, AttributeNameType NAME_A,
          typename TYPE_A, typename... ATTRIBUTE_LIST>
struct FindAttributeList<ATTRIBUTE_NAME,
                         List<Attribute<NAME_A, TYPE_A>, ATTRIBUTE_LIST...>>
    : IF<ATTRIBUTE_NAME != NAME_A,
         FindAttributeList<ATTRIBUTE_NAME, List<ATTRIBUTE_LIST...>>,
         List<Attribute<NAME_A, TYPE_A>, ATTRIBUTE_LIST...>> {};
}  // namespace Implementation

template <AttributeNameType ATTRIBUTE_NAME, typename ATTRIBUTE_LIST>
using FindAttributeList =
    typename Implementation::FindAttributeList<ATTRIBUTE_NAME,
                                               ATTRIBUTE_LIST>::Type;

// FIND ATTRIBUTE TYPE
namespace Implementation {
template <AttributeNameType ATTRIBUTE_NAME, typename ATTRIBUTE_LIST>
struct FindAttributeType {
  static_assert(ATTRIBUTE_NAME != ATTRIBUTE_NAME,
                "ATTRIBUTE_LIST does not contain an attribute with "
                "ATTRIBUTE_NAME as name!");
};

template <AttributeNameType ATTRIBUTE_NAME, AttributeNameType NAME_A,
          typename TYPE_A, typename... ATTRIBUTE_LIST>
struct FindAttributeType<ATTRIBUTE_NAME,
                         List<Attribute<NAME_A, TYPE_A>, ATTRIBUTE_LIST...>>
    : IF<ATTRIBUTE_NAME != NAME_A,
         FindAttributeType<ATTRIBUTE_NAME, List<ATTRIBUTE_LIST...>>,
         ID<TYPE_A>> {};
}  // namespace Implementation

template <AttributeNameType ATTRIBUTE_NAME, typename ATTRIBUTE_LIST>
using FindAttributeType =
    typename Implementation::FindAttributeType<ATTRIBUTE_NAME,
                                               ATTRIBUTE_LIST>::Type;

// HAS DUPLICATE ATTRIBUTE
namespace Implementation {
template <typename ATTRIBUTE_LIST>
struct HasDuplicateAttribute;

template <>
struct HasDuplicateAttribute<List<>> : False {};

template <typename ATTRIBUTE>
struct HasDuplicateAttribute<List<ATTRIBUTE>> : False {};

template <AttributeNameType FIRST_NAME, typename FIRST_TYPE,
          AttributeNameType SECOND_NAME, typename SECOND_TYPE,
          typename... ATTRIBUTE_LIST>
struct HasDuplicateAttribute<
    List<Attribute<FIRST_NAME, FIRST_TYPE>, Attribute<SECOND_NAME, SECOND_TYPE>,
         ATTRIBUTE_LIST...>>
    : IF<FIRST_NAME != SECOND_NAME,
         HasDuplicateAttribute<
             List<Attribute<SECOND_NAME, SECOND_TYPE>, ATTRIBUTE_LIST...>>,
         True> {};
}  // namespace Implementation

template <typename ATTRIBUTE_LIST>
inline constexpr bool HasDuplicateAttribute() {
  return Implementation::HasDuplicateAttribute<
      SortAttributes<ATTRIBUTE_LIST>>::Value;
}
}  // namespace Meta

namespace ImplementationDetail {

template <typename ATTRIBUTE_LIST>
struct AttributeListToString {
  inline static const std::string String = "";
};

template <typename ATTRIBUTE>
struct AttributeListToString<Meta::List<ATTRIBUTE>> {
  inline static const std::string String = ATTRIBUTE::toString();
};

template <typename ATTRIBUTE_A, typename ATTRIBUTE_B, typename... ATTRIBUTES>
struct AttributeListToString<
    Meta::List<ATTRIBUTE_A, ATTRIBUTE_B, ATTRIBUTES...>> {
  inline static const std::string String =
      ATTRIBUTE_A::toString() + ", " +
      AttributeListToString<Meta::List<ATTRIBUTE_B, ATTRIBUTES...>>::String;
};

}  // namespace ImplementationDetail

template <typename ATTRIBUTE_LIST>
inline std::string attributeListToString() noexcept {
  return ImplementationDetail::AttributeListToString<ATTRIBUTE_LIST>::String;
}
