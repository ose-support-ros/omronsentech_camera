time        timestamp       # ROS timestamp when publishing the event
string      module_name     # Source of the event
string      event_name      # Name of the event (empty if from GenICam node)

string      callback_node   # Name of the GenICam callback node 
                            # (empty if from non GenICam node)

string      event_data      # Event data (converted to String datatype)
