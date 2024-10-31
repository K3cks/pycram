import rospy
from json_prolog_msgs.srv import PrologQuery, PrologNextSolution, PrologFinish, PrologQueryRequest, PrologNextSolutionRequest, PrologFinishRequest
from typing import Iterator, Dict, List, Optional

class PrologQueryWrapper:
    def __init__(self, query_str: str, simple_query_srv, next_solution_srv, finish_srv, iterative=True):
        """
        Wraps around ROS Prolog services for a convenient Python interface.
        :param iterative: if False, all solutions will be calculated by rosprolog during the first service call
        """
        self._simple_query_srv = simple_query_srv
        self._next_solution_srv = next_solution_srv
        self._finish_query_srv = finish_srv
        self._finished = False
        self._query_id = None

        # Request to initiate the query
        try:
            request = PrologQueryRequest()
            request.query = query_str
            request.mode = 1 if iterative else 0
            response = self._simple_query_srv(request)
            if not response.ok:
                raise Exception(f'Prolog query failed: {response.message}')
            #self._query_id = response.id
        except rospy.ServiceException as e:
            raise Exception(f"Service call failed: {e}")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.finish()

    def solutions(self) -> Iterator[Dict]:
        """Retrieve solutions from the query one at a time."""
        try:
            while not self._finished:
                request = PrologNextSolutionRequest(id=self._query_id)
                response = self._next_solution_srv(request)
                if response.status == response.OK:
                    yield self._json_to_dict(response.solution)
                elif response.status == response.NO_SOLUTION:
                    break
                else:
                    raise Exception(f"Unknown query status {response.status}")
        finally:
            self.finish()

    def finish(self):
        """Finish the Prolog query and release resources."""
        if not self._finished:
            try:
                request = PrologFinishRequest(id=self._query_id)
                self._finish_query_srv(request)
            except rospy.ServiceException as e:
                rospy.logwarn(f"Failed to finish query: {e}")
            finally:
                self._finished = True

    def _json_to_dict(self, json_text):
        import json
        return json.loads(json_text)

class Prolog:
    def __init__(self, namespace='rosprolog'):
        """Initialize Prolog service clients."""
        rospy.loginfo("[KnowRob] initialize client...")
        rospy.wait_for_service(f'{namespace}/query')
        rospy.wait_for_service(f'{namespace}/next_solution')
        rospy.wait_for_service(f'{namespace}/finish')

        self._simple_query_srv = rospy.ServiceProxy(f'{namespace}/query', PrologQuery)
        self._next_solution_srv = rospy.ServiceProxy(f'{namespace}/next_solution', PrologNextSolution)
        self._finish_query_srv = rospy.ServiceProxy(f'{namespace}/finish', PrologFinish)
        rospy.loginfo("[KnowRob]  done.")

    def query(self, query_str):
        """Execute a Prolog query, yielding solutions iteratively."""
        return PrologQueryWrapper(query_str, simple_query_srv=self._simple_query_srv,
                                  next_solution_srv=self._next_solution_srv, finish_srv=self._finish_query_srv)

    def once(self, query_str: str) -> Optional[Dict]:
        """Retrieve one solution from the query and finish immediately."""
        with PrologQueryWrapper(query_str, self._simple_query_srv, self._next_solution_srv, self._finish_query_srv) as query:
            try:
                return next(query.solutions())
            except StopIteration:
                return None

    def all_solutions(self, query_str: str) -> List[Dict]:
        """Retrieve all solutions at once."""
        with PrologQueryWrapper(query_str, self._simple_query_srv, self._next_solution_srv, self._finish_query_srv, iterative=False) as query:
            return list(query.solutions())
