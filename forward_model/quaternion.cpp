

#include "quaternion.h"


linalg::vector linalg::quaternion::rotate( const vector& v ) const
{
    return ( (*this) *
             quaternion( 0.0, v ) *
             ( this -> conjugate( )) / ( this -> norm( )) ). getvector( );
}


linalg::matrix linalg::quaternion::getrotationmatrix( ) const
{
   // The vectors are the columns of the matrix:

   return matrix( 
              vector( r * r + i * i - j * j - k * k,
                      2.0 * ( i * j + r * k ),
                      2.0 * ( i * k - r * j ) ), 
              vector( 2.0 * ( i * j - r * k ),
                      r * r - i * i + j * j - k * k,
                      2.0 * ( j * k + r * i ) ),
              vector( 2.0 * ( i * k + r * j ),
                      2.0 * ( j * k - r * i ),
                      r * r - i * i - j * j + k * k )) / norm( );
}



std::ostream& linalg::operator << ( std::ostream& stream,
                                    const quaternion& q )
{
   stream << "[ " << q. r << "; ";
   stream << q. i << ", ";
   stream << q. j << ", ";
   stream << q. k << " ]";
   return stream;
}   




