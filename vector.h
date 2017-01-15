
// A three dimensional vector:

#ifndef LINALG_VECTOR_INCLUDED
#define LINALG_VECTOR_INCLUDED   1

#include <iostream>
#include <math.h>

namespace linalg 
{

   struct vector
   {
      double x;
      double y;
      double z;


      vector( )
         : x{0}, y{0}, z{0}
      { }

      vector( double x, double y, double z = 0 )
         : x{x}, y{y}, z{z}
      { }       

      double norm( ) const
      {
         return x*x + y*y + z*z;
      }

      double length( ) const
      {
         return sqrt( norm( ));
      }

   };


   inline vector operator - ( const vector& v )
   {
      return vector( -v.x, -v.y, -v.z );
   }


   inline vector operator + ( const vector& v1, const vector& v2 )
   {
      return vector( v1. x + v2. x, 
                     v1. y + v2. y,
                     v1. z + v2. z );
   }


   inline void operator += ( vector& v1, const vector& v2 )
   {
      v1. x += v2. x;
      v1. y += v2. y;
      v1. z += v2. z; 
   }


   inline vector operator - ( const vector& v1, const vector& v2 )
   {
      return vector( v1. x - v2. x,
                     v1. y - v2. y, 
                     v1. z - v2. z );
   }


   inline void operator -= ( vector& v1, const vector& v2 )
   {
      v1.x -= v2.x;
      v1.y -= v2.y;
      v1.z -= v2.z;
   }


   inline vector operator * ( double d, const vector& v )
   {
      return vector( d * v.x, d * v.y, d * v.z );
   }


   inline vector operator * ( const vector& v, double d )
   {
      return vector( v.x * d, v.y * d, v.z * d );
   }


   inline void operator *= ( vector& v, double d )
   {
      v.x *= d;
      v.y *= d;
      v.z *= d;
   }


   inline vector operator / ( const vector& v, double d )
   {
      return vector( v.x/d, v.y/d, v.z/d );
   }


   inline void operator /= ( vector& v, double d )
   {
      v.x /= d;
      v.y /= d;
      v.z /= d;
   }


   inline double dotproduct( const vector& v1, const vector& v2 )
   {
      return v1.x * v2.x +
             v1.y * v2.y +
             v1.z * v2.z;
   }


   inline vector crossproduct( const vector& v1, const vector& v2 )
   {
      return vector( v1. y * v2. z - v1. z * v2. y,
                     v1. z * v2. x - v1. x * v2. z,
                     v1. x * v2. y - v1. y * v2. x ); 
   }

 
   inline std::ostream& operator << ( std::ostream& stream, const vector& v )
   {
      stream << "[ " << v.x << ", " << v.y << ", " << v.z << " ]";
      return stream;
   }

}

#endif

