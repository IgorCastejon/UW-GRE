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

#ifndef NS3_VECTOR4D_H
#define NS3_VECTOR4D_H

#include "ns3/attribute.h"
#include "ns3/attribute-helper.h"

/**
 * \file
 * \ingroup geometry
 * ns3::Vector4D declaration.
 */

namespace ns3 {

/**
 * \ingroup core
 * \defgroup geometry Geometry primitives
 * \brief Primitives for geometry, such as vectors and angles.
 */
  
/**
 * \ingroup geometry
 * \brief a 4d vector
 * \see attribute_Vector4D
 */
class Vector4D
{
public:
  /**
   * \param [in] _x X coordinate of vector
   * \param [in] _y Y coordinate of vector
   * \param [in] _z Z coordinate of vector
   * \param [in] _w W coordinate of vector
   *
   * Create vector (_x, _y, _z, _w)
   */
  Vector4D (double _x, double _y, double _z, double _w);
  /** Create vector (0.0, 0.0, 0.0, 0.0) */
  Vector4D ();
  
  double x;  //!< x coordinate of vector
  double y;  //!< y coordinate of vector
  double z;  //!< z coordinate of vector
  double w;  //!< w coordinate of vector

  /**
   * Compute the length (magnitude) of the vector.
   * \returns the vector length.
   */
  double GetLength () const;
  
  /**
   * \brief Calculate the Cartesian distance between two points.
   * \param [in] a One point
   * \param [in] b Another point
   * \returns The distance between \p a and \p b.
   */
  friend double CalculateDistance (const Vector4D &a, const Vector4D &b);
  
  /**
   * Output streamer.
   * Vectors are written as "x:y:z:w".
   *
   * \param [in,out] os The stream.
   * \param [in] vector The vector to stream
   * \return The stream.
   */
  friend std::ostream &operator << (std::ostream &os, const Vector4D &vector);

  /**
   * Input streamer.
   *
   * Vectors are expected to be in the form "x:y:z:w".
   *
   * \param [in,out] is The stream.
   * \param [in] vector The vector.
   * \returns The stream.
   */
  friend std::istream &operator >> (std::istream &is, Vector4D &vector);

  /**
   * Less than comparison operator
   * \param [in] a lhs vector
   * \param [in] b rhs vector
   * \returns \c true if \p a is less than \p b
   */
  friend bool operator < (const Vector4D &a, const Vector4D &b);


  friend bool operator == (const Vector4D &a, const Vector4D &b);
  /**
   * Addition operator.
   * \param [in] a lhs vector.
   * \param [in] b rhs vector.
   * \returns The vector sum of \p a and \p b.
   */
  friend Vector4D operator + (const Vector4D &a, const Vector4D &b);

  /**
   * Subtraction operator.
   * \param [in] a lhs vector.
   * \param [in] b rhs vector.
   * \returns The vector difference of \p a and \p b.
   */
  friend Vector4D operator - (const Vector4D &a, const Vector4D &b);
};


ATTRIBUTE_HELPER_HEADER (Vector4D);

  
  
} // namespace ns3

#endif /* NS3_VECTOR4D_H */
