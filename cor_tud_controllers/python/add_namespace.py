"""
Authors: 
    Leandro de Souza Rosa <l.desouzarosa@tudelft.nl>

"""

# import ROS libraris stuff
import rospy, sys
    
if __name__ == '__main__':
    """ Add namespace to the `/namespaces` variable in the ROS param server. If there the variable '/namespaces' is already set, it creates a list and appends the new value.
    """
    try:
        ns = rospy.myargv(argv=sys.argv)[-1]
        ns_var = '/namespaces'

        if not rospy.has_param(ns_var):
            rospy.set_param(ns_var, ns) 
        else:
            ns_list = rospy.get_param(ns_var)
            
            if not isinstance(ns_list, list): 
                ns_list = [ns_list]
            ns_list.append(ns)
            
            rospy.set_param(ns_var, ns_list)
        
    except rospy.ROSInterruptException:
        pass
