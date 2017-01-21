
#ifndef LINALG_ISSINGULAR_INCLUDED
#define LINALG_ISSINGULAR_INCLUDED   1

// 03.04.2012

#include <stdexcept>

namespace linalg
{
   // Thrown when we are unable to invert a matrix due to the fact that
   // it is singular. 

   struct issingular : public std::runtime_error 
   {
      size_t spans; 
      size_t size; 
         // The size of the matrix, and the dimension of the 
         // space that it spans. Inversion is only possible when
         // size == spans.

      issingular( size_t spans, size_t size )
         : runtime_error{ "attempt to invert singular matrix" },
           spans{ spans }, size{ size }
      { }

   };
   
   inline std::ostream& operator << ( std::ostream& stream, 
                                      const issingular& s )
   {
      stream << "matrix of size " << s.size << " is singular\n";
      stream << "it spans a space of dimension " << s.spans << "\n";
      return stream;
   }
}

#endif

