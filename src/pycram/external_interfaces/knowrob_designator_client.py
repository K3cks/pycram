import rospy
from knowrob_designator.srv import SimpleQuery, SimpleQueryRequest, SimpleQueryResponse
import json


# usage examples:
# send_simple_query('type="Milk" AND storagePlace="?storagePlace"')
# send_simple_query('type="Fridge" AND handle="?handle"')
# 'entity (an Object(type="Milk", storagePlace="?storagePlace"))'
# 'entity ( an Object (type="Fridge",location= an Location(handle="?handle" openingState="?open")))'
def send_simple_query(query):
    # Wait until the service is available
    rospy.wait_for_service('simple_query')

    try:
        # Create a service proxy for the 'simple_query' service
        simple_query_service = rospy.ServiceProxy('simple_query', SimpleQuery)

        # Prepare the request with the query
        request = SimpleQueryRequest(query=query)

        # Call the service and get the response
        response = simple_query_service(request)

        # Parse and log the JSON response
        response_data = json.loads(response.response)
        rospy.loginfo(f"Service response: {response_data}")
        return response_data
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None
    

# just a pretty print interface    
def query(query_str):
    send_simple_query(query_str)