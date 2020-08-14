"""
KSTAR Web Service

This service will allow you to submit plans remotely to KSTAR
"""
import os
import json
import uuid
import logging
import shutil
import subprocess
from pathlib import Path
from flask import Flask
from flask_restplus import Api, fields, Resource

app = Flask(__name__)
app.config["VERSION"] = os.getenv("VERSION", "1.0")
app.config["WORK_FOLDER"] = os.getenv("WORK_FOLDER", "/work")

api = Api(
    app,
    version="1.0",
    title="KStar Planner",
    description="This service provides AI planning.",
    default="planners",
    default_label="KStar Planner",
    doc="/",
)

######################################################################
# Set up logging for gunicorn production
######################################################################
app.logger.propagate = False
gunicorn_logger = logging.getLogger("gunicorn.error")
app.logger.handlers = gunicorn_logger.handlers
app.logger.setLevel(gunicorn_logger.level)
TITLE = "  K S T A R   S E R V I C E   {}  ".format(app.config["VERSION"])
app.logger.info(70 * "*")
app.logger.info(TITLE.center(70, "*"))
app.logger.info(70 * "*")


######################################################################
# Define the models so that the docs reflect what can be sent
######################################################################
plan_request = api.model(
    "PlanRequest",
    {
        "domain": fields.String(
            required=True, description="The domain.pddl as a string"
        ),
        "problem": fields.String(
            required=True, description="The problem.pddl as a string"
        ),
        "numplans": fields.Integer(
            required=True, description="The number of plans to return"
        ),
    },
)

plan_data = api.model(
    "PlanData",
    {
        "cost": fields.Integer(required=True, description="The cost of the plan"),
        "actions": fields.List(
            fields.String, required=True, description="The actions for the plan"
        ),
    },
)

plan_response = api.model(
    "PlanResponse",
    {
        "plans": fields.List(
            fields.Nested(plan_data), required=True, description="An array of plans"
        )
    },
)

######################################################################
# Example usage:
# category="topk"
# planner="kstar-topk"
# url=http://localhost:4501/planners/"$category"/"$planner"
# domain=`sed 's/;/\n;/g' domain.pddl | sed '/^;/d' | tr -d '\n'`
# problem=`sed 's/;/\n;/g' problem.pddl | sed '/^;/d' | tr -d '\n'`
# body="{\"domain\": \"$domain\", \"problem\": \"$problem\", \"numplans\":<NUMBER-OF-PLANS>}"
# basebody=`echo $body`
# curl -d "$basebody" -H "Content-Type: application/json" "$url"
######################################################################

######################################################################
#  PATH: /modernization-paths
######################################################################
@api.route("/planners/<category>/<planner>", strict_slashes=False)
class KStarPlanner(Resource):
    """ KStarPlanner

    Creates plans using KStar
    """

    # ------------------------------------------------------------------
    # CREATE A NEW PLAN
    # ------------------------------------------------------------------
    @api.doc("create_plans")
    @api.response(400, "The posted data was not valid")
    @api.response(201, "Plan created successfully")
    @api.expect(plan_request, validate=True)
    @api.marshal_with(plan_response, code=201)
    def post(self, category, planner):
        """ Creates a new Plan

        This endpoint will create a plan based on the data in the body that is posted:
        """
        app.logger.info("Request to create Plan for %s using %s", category, planner)

        # try:

        # Create a unique data folder for the pddl files
        unique_path = uuid.uuid4().hex
        working_folder = os.path.join(app.config["WORK_FOLDER"], unique_path)
        app.logger.info("Creating PDDL folder: %s", working_folder)
        os.mkdir(working_folder)

        # Get the data from the payload and write to a file
        domain_pddl = os.path.join(working_folder, "domain.pddl")
        domain = api.payload["domain"]
        with open(domain_pddl, "w") as writer:
            writer.write(domain)

        problem_pddl = os.path.join(working_folder, "problem.pddl")
        problem = api.payload["problem"]
        with open(problem_pddl, "w") as writer:
            writer.write(problem)

        numplans = api.payload["numplans"]
        plan_file = os.path.join(working_folder, "plan.json")

        # Call the fast forward planner as a process
        search_args = "kstar(blind(),k={0},json_file_to_dump={1})".format(
            numplans, plan_file
        )
        output = subprocess.check_output(
            [
                "/usr/bin/python3",
                "/workspace/kstar/fast-downward.py",
                "--build",
                "release64",
                domain_pddl,
                problem_pddl,
                "--search",
                search_args,
            ],
            stderr=subprocess.STDOUT
        )

        app.logger.info("Process output: %s", output)

        # Read the plan file that was created
        plans = []
        with open(plan_file, "r") as reader:
            plans = json.load(reader)

        # except Exception as err:    # pylint: disable=broad-except
        #     app.logger.error("Could not create Plan: %s", str(err))

        app.logger.info("Returning plan: %s", plans)

        return plans, 201

    # ------------------------------------------------------------------
    # DEELETE OLD PLAN FILES
    # ------------------------------------------------------------------
    @api.doc("delete_plans")
    @api.response(204, "Plan files successfully deleted")
    def delete(self, category, planner):
        """ Delete all work files

        This endpoint will clean out the working folder of all files
        """
        app.logger.info("Cleaning working folder: %s", app.config["WORK_FOLDER"])

        path = Path(app.config["WORK_FOLDER"])
        dirs = [str(x) for x in path.iterdir() if x.is_dir()]
        for name in dirs:
            app.logger.info("- Removing: %s", name)
            shutil.rmtree(name)

        app.logger.info("Clean up complete")
        return "", 204
