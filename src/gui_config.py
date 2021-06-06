FloatLayout:
	canvas:
		Color:
			rgba: 0,0,0,0.8#app.theme_cls.primary_color
		RoundedRectangle:
			pos: self.pos
			size: self.size
			radius: [0, 0, 0, 0]
	#MDLabel:
	#	text: "ROS2 Provenance Record GUI"
	#	halign: "center"
	#	pos_hint: {"center_x": 0.5, "center_y": 0.9}
	#	font_style: "H5"
	#	size_hint: 1,0.1
	#	theme_text_color: "Custom"
	#	text_color: 0, 0, 1, 1
		
		#font_style: "Caption"""
	
	MDBoxLayout:
		orientation: "vertical"

		MDToolbar:
			title: "ROS2 Provenance Record GUI"

		MDLabel:
			text: "."
			halign: "center"

	MDRaisedButton:
		text: "Record"
		text_color: 0, 0, 0, 1
		md_bg_color: 0, 1, 0, 1
		size_hint: 0.2,0.12
		pos_hint: {"center_x": 0.35, "center_y": 0.75}
		on_release: app.trial_run(self, *args)
	MDRaisedButton:
		text: "Stop"
		text_color: 0, 0, 0, 1
		md_bg_color: 1, 0, 0, 0.8
		size_hint: 0.2,0.12
		pos_hint: {"center_x": 0.65, "center_y": 0.75}
		on_release: app.stop_run(self, *args)
	MDRaisedButton:
		text: "Dummy Prov Model"
		text_color: 0, 0, 0, 1
		md_bg_color: 0, 0.8, .9, 1
		size_hint: 0.2,0.12
		pos_hint: {"center_x": 0.35, "center_y": 0.3}
		on_release: app.dummyprov_run(self, *args)
	MDRaisedButton:
		text: "Ros to Prov Parser"
		text_color: 0, 0, 0, 1
		md_bg_color: 0, 0.8, .9, 1
		size_hint: 0.2,0.12
		pos_hint: {"center_x": 0.65, "center_y": 0.3}
		on_release: app.parserprov_run(self, *args)
