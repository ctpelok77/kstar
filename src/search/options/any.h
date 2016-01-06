#ifndef OPTIONS_ANY_H
#define OPTIONS_ANY_H

#include "../utils/memory.h"

#include <algorithm>
#include <exception>
#include <memory>
#include <typeinfo>
#include <type_traits>

/*
  Poor man's version of boost::any, mostly copied from there.
  Does not support
    - construction from literals (e.g., "Any a(3);")
    - moving
    - references as values
    - perfect forwarding.
  These features can be added if needed (see boost::any).
*/

class Any {
    class Placeholder {
public:
        virtual ~Placeholder() {}
        virtual std::unique_ptr<Placeholder> clone() const = 0;
        virtual const std::type_info &type() const = 0;
    };

    template<typename ValueType>
    class Holder : public Placeholder {
        Holder &operator=(const Holder &) = delete;
public:
        ValueType held;

        Holder(const ValueType &value)
            : held(value) {
        }

        virtual std::unique_ptr<Placeholder> clone() const {
            return Utils::make_unique_ptr<Holder<ValueType>>(held);
        }

        virtual const std::type_info &type() const {
            return typeid(ValueType);
        }
    };

    template<typename ValueType>
    friend ValueType *any_cast(Any *);

    std::unique_ptr<Placeholder> content;

public:
    Any() : content(nullptr) {
    }

    Any(const Any &other)
        : content(other.content ? other.content->clone() : nullptr) {
    }

    template<typename ValueType,
             // Disable this constructor for ValueType = Any.
             typename std::enable_if<!std::is_same<ValueType, Any>::value, int>::type = 0>
    Any(ValueType &value)
        : content(Utils::make_unique_ptr<Holder<ValueType>>(value)) {
    }

    ~Any() = default;

    template<typename ValueType,
             // Disable this assignment operator for ValueType = Any.
             typename std::enable_if<!std::is_same<ValueType, Any>::value, int>::type = 0>
    Any &operator=(const ValueType &rhs) {
        Any(rhs).swap(*this);
        return *this;
    }

    Any &operator=(const Any &rhs) {
        Any copied(rhs);
        copied.swap(*this);
        return *this;
    }

    Any &swap(Any &rhs) {
        std::swap(content, rhs.content);
        return *this;
    }

    const std::type_info &type() const {
        return content ? content->type() : typeid(void);
    }
};

class BadAnyCast : public std::bad_cast {
public:
    virtual const char *what() const noexcept override {
        return "BadAnyCast: failed conversion using any_cast";
    }
};


template<typename ValueType>
ValueType *any_cast(Any *operand) {
    if (operand && operand->type() == typeid(ValueType))
        return &static_cast<Any::Holder<ValueType> *>(operand->content.get())->held;
    else
        return nullptr;
}

template<typename ValueType>
inline const ValueType *any_cast(const Any *operand) {
    return any_cast<ValueType>(const_cast<Any *>(operand));
}


template<typename ValueType>
ValueType any_cast(Any &operand) {
    ValueType *result = any_cast<ValueType>(&operand);
    if (!result)
        throw BadAnyCast();
    return *result;
}

template<typename ValueType>
inline ValueType any_cast(const Any &operand) {
    return any_cast<const ValueType>(const_cast<Any &>(operand));
}

/*
This source file was derived from the boost::any library versions 1.45 by
Kevlin Henney. Original copyright statement and license for this original
source follow.

Copyright Kevlin Henney, 2000, 2001, 2002. All rights reserved.
Distributed under the Boost Software License, Version 1.0.

Boost Software License - Version 1.0 - August 17th, 2003

Permission is hereby granted, free of charge, to any person or organization
obtaining a copy of the software and accompanying documentation covered by
this license (the "Software") to use, reproduce, display, distribute,
execute, and transmit the Software, and to prepare derivative works of the
Software, and to permit third-parties to whom the Software is furnished to
do so, all subject to the following:

The copyright notices in the Software and this entire statement, including
the above license grant, this restriction and the following disclaimer,
must be included in all copies of the Software, in whole or in part, and
all derivative works of the Software, unless such copies or derivative
works are solely in the form of machine-executable object code generated by
a source language processor.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/
#endif
