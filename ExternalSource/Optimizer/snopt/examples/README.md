
### A note about the examples
`cppexamples` shows how to compile snopt using the normal headers it came with.

`cppexamples_for_sejong` shows how to compile snopt when using it for the sejong library

The primary difference between the two methods is how snopt is imported. 
ie: `#include "snoptProblem.hpp"` vs `#include <Optimizer/snopt/include/snoptProblem.hpp>` respectively.


### To run the examples for SNOPT
````
$ cd {PATH_TO_sejong}/Sejong_Dynamic_Control_Toolkit/Optimizer/snopt/examples/specs_file_for_examples
````
#### Execute the example scripts:
If `cppexamples_for_sejong` was compiled

````
$ ../../../../build/sj_snopt_cpp_catmixa
$ ../../../../build/sj_snopt_cpp_hs118
$ ../../../../build/sj_snopt_cpp_sntoya
$ ../../../../build/sj_snopt_cpp_sntoyb
$ ../../../../build/sj_snopt_cpp_sntoyc
````
If `cppexamples` was compiled:

````
$ ../../../../build/snopt_cpp_catmixa
$ ../../../../build/snopt_cpp_hs118
$ ../../../../build/snopt_cpp_sntoya
$ ../../../../build/snopt_cpp_sntoyb
$ ../../../../build/snopt_cpp_sntoyc
````
