import xml.etree.ElementTree
 

class InputController():
	"""
	Parent class for all Input Controllers for IRONLab Fetch Robot
	"""

	def __init__(self, spec):
		self.spec_etree = spec
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
		#TODO: parse xml file into dict