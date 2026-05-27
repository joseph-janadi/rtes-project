# rtes-project
Final project for CU Boulder's Real-Time Embedded Systems specialization

## Services

| Service   | WCET (ms) | T (ms)    | D (ms)    | Core  |
| ---       | ---       | ---       | ---       | ---   |
| Read      | 0.25      | 33        | 33        | 2     | 
| Select    | 25        | 200       | 200       | 2     |
| Write     | 65        | 1000      | 1000      | 2     |

$U = \frac{0.25}{33} + \frac{25}{200} + \frac{65}{1000} = 0.1976 = 19.76\%$
