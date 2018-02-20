Welcome to the page of K\* -- a state of the art Top-k planner based on the 
Fast Downward planning system.

### Usage ###

```
./fast-downward.py <domain_file> <problem_file> --search "kstar(heuristic,k)"
```
* _heurisitic_:  any heuristic provided by Fast Downward  
(http://www.fast-downward.org/Doc/Heuristic).   
**Disclaimer**: Optimality of K\* is only guaranteed with an admissible and consistent heuristic.  

* _k_:  number of plans

### Citation ###
Michael Katz, Shirin Sohrabi, Octavian Udrea and Dominik Winterer  
**A Novel Iterative Approach to Top-k Planning** [(bib)](/top_k.bib)  
*In ICAPS 2018*  
*To appear*