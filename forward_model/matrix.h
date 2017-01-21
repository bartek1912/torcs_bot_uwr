
#ifndef LINALG_MATRIX_INCLUDED
#define LINALG_MATRIX_INCLUDED   1

// Written by Hans de Nivelle, 2nd Christmas day 2009.
// Made some changes on december 15th 2010. 
// Made more changes on august 30th 2012. 

#include <iostream>

#include "vector.h"
#include "issingular.h"

namespace linalg
{
   class matrix 
   {
      double repr [3][3];


      // v1 goes into the first column, v2 into the second, and
      // v3 into the last column.
      // One can also say: v1 is the image of (1,0,0), 
      //                   v2 is the image of (0,1,0),
      //                   v3 is the image of (0,0,1). 

   public: 
      matrix( const vector& v1, const vector& v2, const vector& v3 )
      {
         repr[0][0] = v1. x; repr [1][0] = v1. y; repr [2][0] = v1. z;  
         repr[0][1] = v2. x; repr [1][1] = v2. y; repr [2][1] = v2. z;  
         repr[0][2] = v3. x; repr [1][2] = v3. y; repr [2][2] = v3. z;  
      }  

   public: 
      static matrix zero( );
      static matrix identity( ); 
      static matrix scaling( double s );
         // A diagonal matrix with s on the diagonal. 
      static matrix scaling( vector s );
         // A diagonal matrix with s.x,s.y,s.z on the diagonal. 
      static matrix rotation( double d );
         // A rotation by angle d (in rad) around Z-axis.
      static matrix rotation( vector r );
         // A rotation around vector r. The amount of rotation is
         // determined by |r|.

      double determinant( ) const;

      vector operator( ) ( vector v ) const; 

      matrix transpose( ) const; 
      matrix inverse( double border ) const; 
         // border is the border below which pivots will be considered 
         // zero during sweeping. If you are absolutely certain that 
         // the matrix has an inverse, then using border = 0.0 is fine.
         // If the matrix has no inverse, this may go undetected
         // due to rounding errors, and the result is a nonsense matrix.
         // This can be avoided by setting border = 1.0E-12. 

      static matrix crossproduct( const vector& v );
         // Returns the matrix M that represents the left cross product
         // with v. For every vector x, we have
         //    M(x) = v X x.

      matrix& composewith( const matrix& second );
         // The current matrix is replaced by a new matrix that
         // first carries out the function belonging to current matrix, 
         // and then the function belonging to the second matrix. 
         // (*this := second * *this.) 

      friend matrix operator - ( const matrix& m );
      friend matrix operator + ( const matrix& m1, const matrix& m2 );
      friend void operator += ( matrix& m1, const matrix& m2 );

      friend matrix operator - ( const matrix& m1, const matrix& m2 );
      friend void operator -= ( matrix& m1, const matrix& m2 );

      friend void operator *= ( matrix& m, double d );
      friend matrix operator * ( const matrix& m, double d );
      friend matrix operator * ( double d, const matrix& m ); 

      friend void operator /= ( matrix& m, double d );
      friend matrix operator / ( const matrix& m, double ); 

      friend matrix operator * ( const matrix& m1, const matrix& m2 );
      friend vector operator * ( const matrix& m, const vector& v );

      friend std::ostream& operator << ( std::ostream& , const matrix& ); 

      friend class fourmatrix; 
   };

   matrix operator - ( const matrix& m ); 

   matrix operator + ( const matrix& m1, const matrix& m2 );
   void operator += ( matrix& m1, const matrix& m2 );

   matrix operator - ( const matrix& m1, const matrix& m2 );
   void operator -= ( matrix& m1, const matrix& m2 );
   
   void operator *= ( matrix& m, double d );
   matrix operator * ( const matrix& m, double d );
   inline matrix operator * ( double d, const matrix& m ) { return m*d; }

   void operator /= ( matrix& m, double d ); 
   matrix operator / ( const matrix& m, double );

   matrix operator * ( const matrix& m1, const matrix& m2 );

   inline vector operator * ( const matrix& m, const vector& v )
      { return m(v); } 

   inline matrix& matrix::composewith( const matrix& second )
      { *this = second * (*this); return *this; }

   std::ostream& operator << ( std::ostream& stream, const matrix& m ); 
}

#endif

