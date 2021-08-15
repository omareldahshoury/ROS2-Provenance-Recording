FloatLayout:
	canvas:
		Color:
			rgba: 0,0,0,0.8#app.theme_cls.primary_color
		RoundedRectangle:
			pos: self.pos
			size: self.size
			radius: [0, 0, 0, 0]

	
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
		pos_hint: {"center_x": 0.5, "center_y": 0.60}
		on_release: app.record(self, *args)
	
	MDRaisedButton:
		text: "Dummy Prov Model"
		text_color: 0, 0, 0, 1
		md_bg_color: 0, 0.8, .9, 1
		size_hint: 0.2,0.12
		pos_hint: {"center_x": 0.35, "center_y": 0.3}
		on_release: app.dummyprov_run(self, *args)
	MDRaisedButton:
		text: "Ros Snapshot"
		text_color: 0, 0, 0, 1
		md_bg_color: 0, 0.8, .9, 1
		size_hint: 0.2,0.12
		pos_hint: {"center_x": 0.65, "center_y": 0.3}
		on_release: app.ROS_Snapshot(self, *args)
