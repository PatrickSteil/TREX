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

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "../../Helpers/Assert.h"
#include "../../Helpers/IO/Serialization.h"
#include "../../Helpers/Meta.h"
#include "../../Helpers/String/Enumeration.h"
#include "../../Helpers/Vector/Vector.h"
#include "Attribute.h"
#include "AttributeHandle.h"
#include "AttributeNames.h"
using Meta::List;

template <typename ATTRIBUTE_LIST>
class AttributeRecord;

template <>
class AttributeRecord<List<>> {
 public:
  using Type = AttributeRecord<List<>>;
  inline constexpr static size_t NumberOfAttributes = 0;

  template <AttributeNameType ATTRIBUTE_NAME>
  using AttributeType =
      typename Meta::FindAttributeType<ATTRIBUTE_NAME, List<>>;

  template <AttributeNameType ATTRIBUTE_NAME>
  inline constexpr static bool HasAttribute(
      const AttributeNameWrapper<ATTRIBUTE_NAME>) noexcept {
    return false;
  }

 public:
  AttributeRecord() {}
  AttributeRecord(const Type&) {}
  AttributeRecord(Type&&) {}
  Type& operator=(const Type&) noexcept { return *this; }
  Type& operator=(Type&&) noexcept { return *this; }

  template <typename OTHER>
  AttributeRecord(const OTHER&) {}
  template <typename OTHER>
  AttributeRecord(OTHER&&) {}
  template <typename OTHER>
  Type& operator=(const OTHER&) noexcept {
    return *this;
  }
  template <typename OTHER>
  Type& operator=(OTHER&&) noexcept {
    return *this;
  }

  AttributeRecord(const Type&, const Type&) {}
  template <typename ATTRIBUTES>
  AttributeRecord(const ATTRIBUTES&, const size_t) {}

  template <typename FUNCTION>
  inline void forEach(FUNCTION&) noexcept {}
  template <typename FUNCTION>
  inline void forEach(const FUNCTION&) const noexcept {}

  inline std::string toString() const noexcept { return ""; }

 protected:
  inline void toString(Enumeration&) const noexcept {}
};

template <typename ATTRIBUTE, typename... ATTRIBUTE_LIST>
class AttributeRecord<List<ATTRIBUTE, ATTRIBUTE_LIST...>>
    : public AttributeRecord<List<ATTRIBUTE_LIST...>> {
 public:
  template <typename T>
  friend class AttributeRecord;

 private:
  using Attribute = ATTRIBUTE;
  using Super = AttributeRecord<List<ATTRIBUTE_LIST...>>;
  using ValueType = typename Attribute::Type;
  using AttributeList = List<ATTRIBUTE, ATTRIBUTE_LIST...>;
  inline constexpr static AttributeNameWrapper<Attribute::Name> ThisAttribute =
      AttributeNameWrapper<Attribute::Name>();

  template <AttributeNameType ATTRIBUTE_NAME>
  using SuperByName =
      AttributeRecord<Meta::FindAttributeList<ATTRIBUTE_NAME, AttributeList>>;

  template <typename T, typename = void>
  struct IsApplicable : Meta::False {};

  template <typename T>
  struct IsApplicable<T, decltype(std::declval<T>()(std::declval<ValueType&>()),
                                  void())> : Meta::True {};

  template <typename T, typename = void>
  struct IsApplicableWithName : Meta::False {};

  template <typename T>
  struct IsApplicableWithName<T, decltype(std::declval<T>()(
                                              std::declval<ValueType&>(),
                                              AttributeNameType()),
                                          void())> : Meta::True {};

 public:
  using Type = AttributeRecord<AttributeList>;
  inline constexpr static size_t NumberOfAttributes =
      Super::NumberOfAttributes + 1;

  template <AttributeNameType ATTRIBUTE_NAME>
  using AttributeType =
      typename Meta::FindAttributeType<ATTRIBUTE_NAME, AttributeList>;

  template <AttributeNameType ATTRIBUTE_NAME>
  inline constexpr static bool HasAttribute(
      const AttributeNameWrapper<ATTRIBUTE_NAME>) noexcept {
    return Meta::ContainsAttribute<ATTRIBUTE_NAME, AttributeList>();
  }

  inline constexpr static std::array<AttributeNameType,
                                     sizeof...(ATTRIBUTE_LIST) + 1>
      AttributeNames = {ATTRIBUTE::Name, ATTRIBUTE_LIST::Name...};

 public:
  AttributeRecord() : Super(), value() {}
  AttributeRecord(ValueType value, typename ATTRIBUTE_LIST::Type... values)
      : Super(values...), value(value) {}
  AttributeRecord(const Type& other)
      : Super(static_cast<const Super&>(other)), value(other.value) {}
  AttributeRecord(Type&& other)
      : Super(std::move(static_cast<Super&&>(other))),
        value(std::move(other.value)) {}
  Type& operator=(const Type& other) noexcept {
    Super::operator=(static_cast<const Super&>(other));
    value = other.value;
    return *this;
  }
  Type& operator=(Type&& other) noexcept {
    Super::operator=(std::move(static_cast<Super&&>(other)));
    value = std::move(other.value);
    return *this;
  }

  template <typename OTHER>
  AttributeRecord(const AttributeRecord<OTHER>& other) : Super(other) {
    if constexpr (AttributeRecord<OTHER>::HasAttribute(ThisAttribute)) {
      value = other[ThisAttribute];
    } else {
      value = ValueType();
    }
  }
  template <typename OTHER>
  AttributeRecord(AttributeRecord<OTHER>&& other) : Super(std::move(other)) {
    if constexpr (AttributeRecord<OTHER>::HasAttribute(ThisAttribute)) {
      value = std::move(other[ThisAttribute]);
    } else {
      value = ValueType();
    }
  }
  template <typename OTHER>
  Type& operator=(const OTHER& other) noexcept {
    Super::operator=(other);
    if constexpr (AttributeRecord<OTHER>::HasAttribute(ThisAttribute)) {
      value = other[ThisAttribute];
    } else {
      value = ValueType();
    }
    return *this;
  }
  template <typename OTHER>
  Type& operator=(OTHER&& other) noexcept {
    Super::operator=(std::move(other));
    if constexpr (AttributeRecord<OTHER>::HasAttribute(ThisAttribute)) {
      value = std::move(other[ThisAttribute]);
    } else {
      value = ValueType();
    }
    return *this;
  }

  AttributeRecord(const Type& a, const Type& b)
      : Super(static_cast<const Super&>(a), static_cast<const Super&>(b)) {
    if constexpr (std::is_integral<ValueType>::value ||
                  std::is_floating_point<ValueType>::value) {
      value = a.value + b.value;
    }
  }
  template <typename ATTRIBUTES>
  AttributeRecord(const ATTRIBUTES& attributes, const size_t index)
      : Super(attributes, index) {
    if constexpr (ATTRIBUTES::HasAttribute(ThisAttribute)) {
      value = attributes[ThisAttribute][index];
    }
  }

  template <typename FUNCTION>
  inline void forEach(const FUNCTION& function) noexcept {
    if constexpr (IsApplicableWithName<FUNCTION>::Value) {
      function(value, ThisAttribute);
    } else if constexpr (IsApplicable<FUNCTION>::Value) {
      function(value);
    }
    Super::forEach(function);
  }
  template <typename FUNCTION>
  inline void forEach(const FUNCTION& function) const noexcept {
    if constexpr (IsApplicableWithName<FUNCTION>::Value) {
      function(value, ThisAttribute);
    } else if constexpr (IsApplicable<FUNCTION>::Value) {
      function(value);
    }
    Super::forEach(function);
  }

  template <AttributeNameType ATTRIBUTE_NAME>
  inline const AttributeType<ATTRIBUTE_NAME>& operator[](
      const AttributeNameWrapper<ATTRIBUTE_NAME>) const noexcept {
    static_assert(HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()),
                  "Current instantiation of AttributeRecord<List<ATTRIBUTE, "
                  "ATTRIBUTE_LIST...>> does not contain an Attribute named "
                  "ATTRIBUTE_NAME");
    return SuperByName<ATTRIBUTE_NAME>::value;
  }
  template <AttributeNameType ATTRIBUTE_NAME>
  inline AttributeType<ATTRIBUTE_NAME>& operator[](
      const AttributeNameWrapper<ATTRIBUTE_NAME>) noexcept {
    static_assert(HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()),
                  "Current instantiation of AttributeRecord<List<ATTRIBUTE, "
                  "ATTRIBUTE_LIST...>> does not contain an Attribute named "
                  "ATTRIBUTE_NAME");
    return SuperByName<ATTRIBUTE_NAME>::value;
  }

  template <AttributeNameType ATTRIBUTE_NAME>
  inline const AttributeType<ATTRIBUTE_NAME>& get(
      const AttributeNameWrapper<ATTRIBUTE_NAME>) const noexcept {
    static_assert(HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()),
                  "Current instantiation of AttributeRecord<List<ATTRIBUTE, "
                  "ATTRIBUTE_LIST...>> does not contain an Attribute named "
                  "ATTRIBUTE_NAME");
    return SuperByName<ATTRIBUTE_NAME>::value;
  }
  template <AttributeNameType ATTRIBUTE_NAME>
  inline AttributeType<ATTRIBUTE_NAME>& get(
      const AttributeNameWrapper<ATTRIBUTE_NAME>) noexcept {
    static_assert(HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()),
                  "Current instantiation of AttributeRecord<List<ATTRIBUTE, "
                  "ATTRIBUTE_LIST...>> does not contain an Attribute named "
                  "ATTRIBUTE_NAME");
    return SuperByName<ATTRIBUTE_NAME>::value;
  }

  template <AttributeNameType ATTRIBUTE_NAME>
  inline void set(const AttributeNameWrapper<ATTRIBUTE_NAME>,
                  const AttributeType<ATTRIBUTE_NAME>& value) noexcept {
    static_assert(HasAttribute(AttributeNameWrapper<ATTRIBUTE_NAME>()),
                  "Current instantiation of AttributeRecord<List<ATTRIBUTE, "
                  "ATTRIBUTE_LIST...>> does not contain an Attribute named "
                  "ATTRIBUTE_NAME");
    SuperByName<ATTRIBUTE_NAME>::value = value;
  }

  inline std::string toString() const noexcept {
    Enumeration result;
    toString(result);
    return result;
  }

 protected:
  inline void toString(Enumeration& enumeration) const noexcept {
    enumeration << Attribute::String << ": " << value << sep;
    Super::toString(enumeration);
  }

 protected:
  ValueType value{};
};
