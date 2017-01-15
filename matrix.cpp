

#include "matrix.h"
#include "quaternion.h" 
#include <cstdio>
#include <stdexcept>


linalg::matrix linalg::matrix::zero( )
{
   return matrix( vector( 0.0, 0.0, 0.0 ),
                  vector( 0.0, 0.0, 0.0 ),
                  vector( 0.0, 0.0, 0.0 )); 
}


linalg::matrix linalg::matrix::identity( )
{
   return matrix( vector( 1.0, 0.0, 0.0 ),
                  vector( 0.0, 1.0, 0.0 ),
                  vector( 0.0, 0.0, 1.0 ));
}


linalg::matrix linalg::matrix::scaling( double s )
{
   return matrix( vector( s, 0, 0 ),
                  vector( 0, s, 0 ),
                  vector( 0, 0, s ));
}


linalg::matrix linalg::matrix::scaling( vector s )
{
   return matrix( vector( s.x, 0.0, 0.0 ),
                  vector( 0.0, s.y, 0.0 ),
                  vector( 0.0, 0.0, s.z ));
}


linalg::matrix linalg::matrix::rotation( double d )
{
   return matrix( vector( cos(d), sin(d), 0.0 ),
                  vector( -sin(d), cos(d), 0.0 ),
                  vector( 0.0, 0.0, 1.0 ));
}


linalg::matrix linalg::matrix::rotation( vector r )
{
   return quaternion::rotation(r). getrotationmatrix( );
}


double linalg::matrix::determinant( ) const
{
   return ( repr [0][0] * repr [1][1] * repr [2][2] +
            repr [1][0] * repr [2][1] * repr [0][2] +
            repr [2][0] * repr [0][1] * repr [1][2] ) -
          ( repr [0][0] * repr [2][1] * repr [1][2] +
            repr [1][0] * repr [0][1] * repr [2][2] +
            repr [2][0] * repr [1][1] * repr [0][2] );
}


linalg::vector linalg::matrix::operator( ) ( vector v ) const
{
   return {
      repr [0][0] * v.x + repr [0][1] * v.y + repr [0][2] * v.z,
      repr [1][0] * v.x + repr [1][1] * v.y + repr [1][2] * v.z,
      repr [2][0] * v.x + repr [2][1] * v.y + repr [2][2] * v.z };
}


linalg::matrix linalg::matrix::transpose( ) const
{
   return { { repr[0][0], repr[0][1], repr[0][2] },
            { repr[1][0], repr[1][1], repr[1][2] },
            { repr[2][0], repr[2][1], repr[2][2] } };
}


linalg::matrix linalg::matrix::inverse( double border ) const 
{
   matrix m = *this;
   matrix inv = matrix::identity( );
   unsigned int failures = 0;
   if( border < 0.0 )
      throw std::runtime_error( "border must be >= 0" );

   for( unsigned int j = 0; j < 3; ++ j )
   {
      // We determine the row i (j <= i < 3) on which m.repr [i][j] 
      // has the maximal absolute value: 

      double maxabs = border; 
      unsigned int maxi = 0;
      for( unsigned int i = j; i < 3; ++ i )
      {
         if( fabs( m. repr [i] [j] ) > maxabs )
         {
            maxabs = fabs( m. repr [i] [j] );
            maxi = i;
         }
      }

      // If we have maxabs == border, then the matrix is singular. 

      if( maxabs == border )
         failures = failures + 1; 
      else
      { 
         // If maxi != j, then we exchange the rows maxi and j. 

         if( maxi != j )
         {
            for( unsigned int k = 0; k < 3; ++ k )
            {
               double x = m. repr [ maxi ] [k];
               m. repr [ maxi ] [k] = m. repr [j] [k];
               m. repr [j] [k] = x;

               x = inv. repr [ maxi ] [k];
               inv. repr [ maxi ] [k] = inv. repr [j] [k];
               inv. repr [j] [k] = x;
            }
         }

         // We use the j-th row to clean the other rows.

         for( unsigned int i = 0; i < 3; ++ i )
         {
            if( i != j )
            {
               double p = m. repr [i][j] / m. repr [j][j];
               for( unsigned int k = 0; k < 3; ++ k )
               {
                  m. repr[i][k] -= p * m. repr [j][k];
                  inv. repr [i][k] -= p * inv. repr [j][k];
               }
            }
         }
      }
   }

   if( failures ) throw issingular( 3 - failures, 3 );

   for( unsigned int i = 0; i < 3; ++ i )
   {
      double p = 1.0 / m. repr [i][i];
      for( unsigned int k = 0; k < 3; ++ k )
      {
         // m. repr [i][k] *= p; // We don't do it anymore, 
                                 // because m will be thrown away anyway.  
         inv. repr [i][k] *= p;
      }
   }
    
   return inv;
}


linalg::matrix linalg::matrix::crossproduct( const vector& v )
{
   return matrix( vector( 0,       v. z,    - v. y ),
                  vector( - v. z,  0,        v. x ),
                  vector( v. y,    - v. x,   0 ));
}


linalg::matrix linalg::operator - ( const matrix& m )
{
   return matrix(
      vector( -m. repr[0][0], -m. repr[1][0], -m. repr[2][0] ),
      vector( -m. repr[0][1], -m. repr[1][1], -m. repr[2][1] ),
      vector( -m. repr[0][2], -m. repr[1][2], -m. repr[2][2] ));
}


linalg::matrix linalg::operator + ( const matrix& m1, const matrix& m2 )
{
   return matrix( 
      vector( m1. repr[0][0] + m2. repr[0][0],
              m1. repr[1][0] + m2. repr[1][0],
              m1. repr[2][0] + m2. repr[2][0] ),
      vector( m1. repr[0][1] + m2. repr[0][1],
              m1. repr[1][1] + m2. repr[1][1],
              m1. repr[2][1] + m2. repr[2][1] ),
      vector( m1. repr[0][2] + m2. repr[0][2],
              m1. repr[1][2] + m2. repr[1][2],
              m1. repr[2][2] + m2. repr[2][2] ));
}


void linalg::operator += ( matrix& m1, const matrix& m2 )
{
   m1. repr [0][0] += m2. repr [0][0];
   m1. repr [0][1] += m2. repr [0][1];
   m1. repr [0][2] += m2. repr [0][2];
   m1. repr [1][0] += m2. repr [1][0];
   m1. repr [1][1] += m2. repr [1][1];
   m1. repr [1][2] += m2. repr [1][2];
   m1. repr [2][0] += m2. repr [2][0];
   m1. repr [2][1] += m2. repr [2][1];
   m1. repr [2][2] += m2. repr [2][2];
}


linalg::matrix linalg::operator - ( const matrix& m1, const matrix& m2 )
{
   return matrix(
      vector( m1. repr[0][0] - m2. repr[0][0],
              m1. repr[1][0] - m2. repr[1][0],
              m1. repr[2][0] - m2. repr[2][0] ),
      vector( m1. repr[0][1] - m2. repr[0][1],
              m1. repr[1][1] - m2. repr[1][1],
              m1. repr[2][1] - m2. repr[2][1] ),
      vector( m1. repr[0][2] - m2. repr[0][2],
              m1. repr[1][2] - m2. repr[1][2],
              m1. repr[2][2] - m2. repr[2][2] ));
}


void linalg::operator -= ( matrix& m1, const matrix& m2 )
{
   m1. repr [0][0] -= m2. repr [0][0];
   m1. repr [0][1] -= m2. repr [0][1];
   m1. repr [0][2] -= m2. repr [0][2];
   m1. repr [1][0] -= m2. repr [1][0];
   m1. repr [1][1] -= m2. repr [1][1];
   m1. repr [1][2] -= m2. repr [1][2];
   m1. repr [2][0] -= m2. repr [2][0];
   m1. repr [2][1] -= m2. repr [2][1];
   m1. repr [2][2] -= m2. repr [2][2];
}


void linalg::operator *= ( matrix& m, double d )
{
   m. repr [0][0] *= d;
   m. repr [0][1] *= d;
   m. repr [0][2] *= d;
   m. repr [1][0] *= d;
   m. repr [1][1] *= d;
   m. repr [1][2] *= d;
   m. repr [2][0] *= d;
   m. repr [2][1] *= d;
   m. repr [2][2] *= d;
}


linalg::matrix linalg::operator * ( const matrix& m, double d )
{
   return matrix( 
      vector( d * m. repr [0][0], 
              d * m. repr [1][0], 
              d * m. repr [2][0] ), 
      vector( d * m. repr [0][1],
              d * m. repr [1][1],
              d * m. repr [2][1] ),
      vector( d * m. repr [0][2],
              d * m. repr [1][2],
              d * m. repr [2][2] )); 
}


void linalg::operator /= ( matrix& m, double d )
{
   m. repr [0][0] /= d;
   m. repr [0][1] /= d;
   m. repr [0][2] /= d;
   m. repr [1][0] /= d;
   m. repr [1][1] /= d;
   m. repr [1][2] /= d;
   m. repr [2][0] /= d;
   m. repr [2][1] /= d;
   m. repr [2][2] /= d;
}


linalg::matrix linalg::operator / ( const matrix& m, double d )
{
   return matrix(
      vector( m. repr [0][0] / d,
              m. repr [1][0] / d,
              m. repr [2][0] / d ),
      vector( m. repr [0][1] / d,
              m. repr [1][1] / d,
              m. repr [2][1] / d ),
      vector( m. repr [0][2] / d,
              m. repr [1][2] / d,
              m. repr [2][2] / d ));
}


linalg::matrix linalg::operator * ( const matrix& m1, const matrix& m2 )
{
   return matrix( 
      vector( m1. repr [0][0] * m2. repr [0][0] +
              m1. repr [0][1] * m2. repr [1][0] +
              m1. repr [0][2] * m2. repr [2][0], 
              m1. repr [1][0] * m2. repr [0][0] +
              m1. repr [1][1] * m2. repr [1][0] +
              m1. repr [1][2] * m2. repr [2][0], 
              m1. repr [2][0] * m2. repr [0][0] +
              m1. repr [2][1] * m2. repr [1][0] +
              m1. repr [2][2] * m2. repr [2][0] ),
      vector( m1. repr [0][0] * m2. repr [0][1] +
              m1. repr [0][1] * m2. repr [1][1] +
              m1. repr [0][2] * m2. repr [2][1],
              m1. repr [1][0] * m2. repr [0][1] +
              m1. repr [1][1] * m2. repr [1][1] +
              m1. repr [1][2] * m2. repr [2][1],
              m1. repr [2][0] * m2. repr [0][1] +
              m1. repr [2][1] * m2. repr [1][1] +
              m1. repr [2][2] * m2. repr [2][1] ),
      vector( m1. repr [0][0] * m2. repr [0][2] +
              m1. repr [0][1] * m2. repr [1][2] +
              m1. repr [0][2] * m2. repr [2][2],
              m1. repr [1][0] * m2. repr [0][2] +
              m1. repr [1][1] * m2. repr [1][2] +
              m1. repr [1][2] * m2. repr [2][2],
              m1. repr [2][0] * m2. repr [0][2] +
              m1. repr [2][1] * m2. repr [1][2] +
              m1. repr [2][2] * m2. repr [2][2] ));
}


std::ostream& linalg::operator << ( std::ostream& stream,
                                    const matrix& m )
{
   for( unsigned int i = 0; i < 3; ++ i )
   {
      if( i == 0 )
         stream << " /  ";
      if( i == 1 )
         stream << " |  ";
      if( i == 2 )
         stream << " \\  ";

      printf( "%.10lf      %.10lf      %.10lf",  
               m. repr [i][0], m. repr [i][1], m. repr [i][2] ); 

      if( i == 0 )
         stream << "  \\\n";
      if( i == 1 )
         stream << "  |\n";
      if( i == 2 ) 
         stream << "  /\n";
   }
   stream << "\n";
   return stream;
}

 
