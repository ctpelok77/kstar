# Top-k Planner Service

Welcome to a fork of the page of K\* planner -- a state of the art Top-k planner integrating the K\* algorithm into Fast Downward.

The original repo is [ctpelok77/kstar](https://github.com/ctpelok77/kstar). This fork adds a REST API service so that it can be called remotely.

## Running the service

Run the service locally via:
```
docker run -d -p 4501:4501 \
    --env server__hostname=`hostname`:4501 \
    --name <YOUR_CONTAINER_NAME> rofrano/kstar
```

A couple of notes:

- This will make the service available on your local machine on port `4501`. See the Docker User Guide for manipulating port mappings and binding to specific interfaces.
- `<YOUR_CONTAINER_NAME>` can be any name like `kstar` or `planner`. It allows you to more easily manipulated it with commands like `docker stop kstar`, `docker start kstar`, and `docker rm kstar`
- The `--env` options are only necessary for the swagger UI available at (http://localhost:4501/api-docs) to properly bind to your physical host name and port instead of the container name. If you do not plan to use the swagger UI, you can safely skip it.

## Using it as a service

The REST API was intentionally kept consistent with the `ctpelok77/ibmresearchaiplanningsolver` API even though it only containes the kstar planner. It should allow easily swapping this planner out for the more robust version.

You can use the planner service with this curl command:

```
url=http://localhost:4501/planners/topk/kstar-topk
domain=`sed 's/;/\n;/g' <DOMAIN-FILE> | sed '/^;/d' | tr -d '\n'`
problem=`sed 's/;/\n;/g' <PROBLEM-FILE> | sed '/^;/d' | tr -d '\n'`
body="{\"domain\": \"$domain\", \"problem\": \"$problem\", \"numplans\":<NUMBER-OF-PLANS>}"
basebody=`echo $body`
curl -d "$basebody" -H "Content-Type: application/json" "$url"
```

Where `<DOMAIN-FILE>` is your `domain.pddl` and `<PROBLEM-FILE>` is your `problem.pddl` and `<NUMBER-OF-PLANS>` is a positive integer that represents the number of plans you want to generate.

The response will be a `json` structure with all of the plans. Using the gripper example you would get something like this:

```
{
    "plans": [
        {
            "cost": 11,
            "actions": [
                "pick ball2 rooma left",
                "pick ball1 rooma right",
                "move rooma roomb",
                "drop ball1 roomb right",
                "drop ball2 roomb left",
                "move roomb rooma",
                "pick ball3 rooma left",
                "pick ball4 rooma right",
                "move rooma roomb",
                "drop ball3 roomb left",
                "drop ball4 roomb right"
            ]
        }
    ]
}
``` 

## Running locally

You can run the Docker image locally by sharing your current folder and using a `bash` shell inside the container like this:

```
docker run --rm -it -v $(pwd):/work rofrano/kstar bash
```

## Usage

```
/workspace/kstar/fast-downward.py --build release64 <domain_file> <problem_file> --search "kstar(heuristic,k=<number-of-plans>)"
```

For example (using files from [git repo](https://github.com/rofrano/kstar)):
```
/workspace/kstar/fast-downward.py --build release64 examples/gripper/domain.pddl examples/gripper/prob01.pddl --search "kstar(blind(),k=100)"
```

* _heurisitic_:  any heuristic provided by Fast Downward  
(http://www.fast-downward.org/Doc/Heuristic).   
**Disclaimer**: Optimality of K\* is only guaranteed with an admissible and consistent heuristic.  


### Citation ###
Michael Katz, Shirin Sohrabi, Octavian Udrea and Dominik Winterer  
**A Novel Iterative Approach to Top-k Planning** [[pdf]](https://www.aaai.org/ocs/index.php/ICAPS/ICAPS18/paper/download/17749/16971) [[bib]](/top_k.bib)  
*In ICAPS 2018*  

### Contact ###
For questions and comments please get in touch with Michael Katz (michael.katz1@ibm.com).