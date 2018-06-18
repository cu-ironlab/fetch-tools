import xml.etree.ElementTree
 

class InputController():
	"""
	Parent class for all Input Controllers for IRONLab Fetch Robot
	"""

	def __init__(self, spec):
		self.spec_etree = spec.getroot()
		self.create_control_dict()

	def create_control_dict(self):
		"""
		Creates dict that will be updated with control signal inputs
		Dict filled using specifications given in spec_etree

		TODO: move the following xml reference somewhere else
		<?xml version="1.0"?>
		<data>
			<input name='forward'>
				<type>float</type>
			</input>
			<input name='command'>
				<type>string</type>
			</input>
			.
			.
			.
		</data>
		"""
		self.control_dict = {}
		for input_signal in self.spec_etree.iter('input'):
			input_name = input_signal.attrib['name']
			inputs = []
			input_types = []
			for x_in in input_signal.iter('type'):
				input_types.append(x_in.text)
				inputs.append(None)
			self.control_dict[input_name]["types"] = input_types
			self.control_dict[input_name]["inputs"] = inputs

	def update_control_signal(self, signal_name, signal_update):
		self.control_dict[signal_name]["inputs"] = signal_update
		#check for input type match here? or somewhere else?

	def get_all_controls(self):
		return self.control_dict

	def get_control_signal(self, signal_name):
		return self.control_dict[signal_name]