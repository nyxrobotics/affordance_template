import yaml
import rospy

class RecognitionObject(object) :
    def __init__(self) :
        self.reset()

    def reset(self) :
        self.type = ""
        self.package =  ""
        self.launch_file = ""
        self.image_path = ""

    def load_from_file(self, filename) :

        try:
            f = open(filename)
            self.yaml_config = yaml.load(f.read())
            f.close()

            self.type = self.yaml_config['type']
            self.package = str(self.yaml_config['package'])
            self.image_path = str(self.yaml_config['image_path'])
            self.launch_file = self.yaml_config['launch_file']

            self.print_yaml()

            return True

        except :
            rospy.logerr("RecognitionObject::load_from_file() -- error opening config file")
            return False


    def print_yaml(self) :
        if not self.yaml_config == None:
            print "============================="
            print "RecognitionObject Info: "
            print "============================="
            print " type: ", self.yaml_config['type']
            print " package: ", self.yaml_config['package']
            print " image_path: ", self.yaml_config['image_path']
            print " launch_file: ", self.yaml_config['launch_file']
            print "============================="

