from InputController import InputController
import pygame

class KeyboardController(InputController):

	def __init__(self, controller_spec, keyboard_layout):
		InputController.__init__(self, controller_spec)
		self.setup_key_listeners(keyboard_layout)

	def setup_key_listeners(self, keyboard_layout):
		