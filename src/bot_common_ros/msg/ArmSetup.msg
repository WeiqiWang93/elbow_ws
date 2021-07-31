#
#
#
# valid types:
#   set_bounding_box -- data: bbox
#   set_defaults_topic -- data: topic_name
#   use_bounding_box   -- data: flag
#   use_monitor        -- data: flag
#   use_data_publisher -- data: flag
#
string type
string topic_name
visualization_msgs/Marker bbox
bool flag