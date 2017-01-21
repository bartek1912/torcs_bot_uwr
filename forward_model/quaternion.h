
// Written by Hans de Nivelle, 2010.

#ifndef LINALG_QUATERNION_INCLUDED
#define LINALG_QUATERNION_INCLUDED    1


#include <iostream>
#include <math.h>
#include "vector.h"
#include "matrix.h"


namespace linalg
{

   struct quaternion 
   {
      double r;
      double i;
      double j;
      double k;

  
      quaternion( double r, double i, double j, double k )
         : r{r}, i{i}, j{j}, k{k}
      { }

      quaternion( double r ) 
         : r{r}, i{0.0}, j{0.0}, k{0.0}
      { }

      quaternion( double r, vector v )
         : r{r}, i{ v.x }, j{ v.y }, k{ v.z }
      { }

  
      quaternion conjugate( ) const
         { return { r, -i, -j, -k }; }

      double norm( ) const 
         { return r*r + i*i + j*j + k*k; }

      double length( ) const  
         { return sqrt( r*r + i*i + j*j + k*k ); }

      quaternion inverse( ) const
      { 
         double n = norm( );
         return { r / n, -i / n, -j / n, -k / n }; 
      }
      
      vector getvector( ) const
      {
         return { i, j, k };
      }

      
      // Returns a unit quaternion that represents a rotation around
      // axis v, with angle |v|.

      static inline quaternion rotation( const vector& v )
      {
         double l = v. length( );
         if( l == 0.0 )
            return quaternion( 1.0, vector( 0.0, 0.0, 0.0 ));
         else
            return quaternion( cos( 0.5 * l ), 
                               vector( ( v / l ) * sin( 0.5 * l )));
      }



      // Apply the quaternion, as a rotation, on vector v.
      // (This is q.(0;v).q^{-1}. )
      // If you want to make many rotations, then it is better
      // to construct the matrix. 

      linalg::vector rotate( const linalg::vector& v ) const;


      matrix getrotationmatrix( ) const;
         // Construct a matrix for the rotation that is represented by the
         // quaternion. It is the matrix M that satisfies 
         //      M(v) = q. (0;v). q^{-1}.

      quaternion& composewith( const quaternion& q );
         // Replace *this by the quaternion that one obtains
         // when *this is followed by q. 
         //    (*this) := q * (*this). 

   };


   inline quaternion operator - ( const quaternion& q )
   { return quaternion( -q.r, -q.i, -q.j, -q.k ); }


   inline void operator += ( quaternion& q1, const quaternion& q2 )
   {
      q1. r += q2. r;
      q1. i += q2. i;
      q1. j += q2. j;
      q1. k += q2. k;
   }

   inline quaternion operator + ( const quaternion& q1, const quaternion& q2 )
   {
      return quaternion( q1.r + q2.r,
                         q1.i + q2.i,
                         q1.j + q2.j,
                         q1.k + q2.k );
   }

   inline void operator -= ( quaternion& q1, const quaternion& q2 )
   {
      q1. r -= q2. r;
      q1. i -= q2. i;
      q1. j -= q2. j;
      q1. k -= q2. k;
   }

   inline quaternion operator - ( const quaternion& q1, const quaternion& q2 )
   {
      return quaternion( q1.r - q2.r,
                         q1.i - q2.i,
                         q1.j - q2.j,
                         q1.k - q2.k ); 
   }   
  
   inline void operator *= ( quaternion& q, double d )
   {
      q. r *= d;
      q. i *= d;
      q. j *= d;
      q. k *= d;
   }

   inline quaternion operator * ( const quaternion& q, double d )
   {
      return { q.r * d,  q.i * d, q.j * d, q.k * d };
   }

   inline quaternion operator * ( double d, const quaternion& q )
   {
      return { d * q.r,  d * q.i, d * q.j, d * q.k };
   }

   inline void operator /= ( quaternion& q, double d )
   {
      q. r /= d;
      q. i /= d;
      q. j /= d;
      q. k /= d;
   }

   inline quaternion operator / ( const quaternion& q, double d )
   {
      return { q.r / d,  q.i / d, q.j / d, q.k / d };
   }


   // Quaternion multiplication can be used for composing rotations. 
   // ( q1 * q2 ) is the rotation that first performs q2, then q1.  

   inline quaternion operator * ( const quaternion& q1, const quaternion& q2 )
   {
      return quaternion( 
          q1. r * q2. r - ( q1. i * q2. i + q1. j * q2. j + q1. k * q2. k ),

          q1. r * q2. i + q1. i * q2. r + q1. j * q2. k - q1. k * q2. j,
          q1. r * q2. j + q1. j * q2. r + q1. k * q2. i - q1. i * q2. k,
          q1. r * q2. k + q1. k * q2. r + q1. i * q2. j - q1. j * q2. i );
   }
   
   inline quaternion operator / ( const quaternion& q1, const quaternion& q2 )
   {
      return q1 * q2. inverse( ); 
   }

   inline void operator /= ( quaternion& q1, const quaternion& q2 )
   {
      q1 = q1 / q2;
   }

   inline quaternion& quaternion::composewith( const quaternion& q )
      { (*this) = q * (*this); return *this; } 

   std::ostream& operator << ( std::ostream& stream, const quaternion& q );
}

#endif

