/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 Universidade Federal de Minas Gerais
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Igor Castejon F. e Castro <igorcastejon@dcc.ufmg.br>
 */

#include <cmath>
#include <sstream>
#include <tuple>

#include "ns3/fatal-error.h"
#include "ns3/log.h"

#include "vector4D.h"

/**
 * \file
 * \ingroup attribute_Vector
 * ns3::Vector4D attribute value implementations.
 */

namespace ns3
{
    NS_LOG_COMPONENT_DEFINE("Vector4D");

    ATTRIBUTE_HELPER_CPP(Vector4D);

    Vector4D::Vector4D(double _x, double _y, double _z, double _w)
        : x(_x),
          y(_y),
          z(_z),
          w(_w)
    {
        NS_LOG_FUNCTION(this << _x << _y << _z << _w);
    }

    Vector4D::Vector4D()
        : x(0.0),
          y(0.0),
          z(0.0),
          w(0.0)
    {
        NS_LOG_FUNCTION(this);
    }

    double
    Vector4D::GetLength() const
    {
        NS_LOG_FUNCTION(this);
        return std::sqrt(x * x + y * y + z * z + w * w);
    }

    double
    CalculateDistance(const Vector4D &a, const Vector4D &b)
    {
        NS_LOG_FUNCTION(a << b);
        return (b - a).GetLength();
    }

    std::ostream &operator<<(std::ostream &os, const Vector4D &vector)
    {
        os << vector.x << ":" << vector.y << ":" << vector.z << ":" << vector.w;
        return os;
    }

    bool operator==(const Vector4D &a, const Vector4D &b)
    {
        return ((a.x == b.x) && (a.y == b.y) && (a.z == b.z) && (a.w == b.w));
    }

    std::istream &operator>>(std::istream &is, Vector4D &vector)
    {
        char c1, c2, c3;
        is >> vector.x >> c1 >> vector.y >> c2 >> vector.z >> c3 >> vector.w;
        if (c1 != ':' ||
            c2 != ':' ||
            c3 != ':') {
            is.setstate(std::ios_base::failbit);
        }
        return is;
    }

    bool operator<(const Vector4D &a, const Vector4D &b)
    {
        return std::tie(a.x, a.y, a.z, a.w) <
               std::tie(b.x, b.y, b.z, b.w);
    }

    Vector4D
    operator+(const Vector4D &a, const Vector4D &b)
    {
        return Vector4D(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
    }

    Vector4D
    operator-(const Vector4D &a, const Vector4D &b)
    {
        return Vector4D(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
    }
}  // namespace ns3