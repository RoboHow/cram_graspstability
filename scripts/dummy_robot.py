#!/usr/bin/env python

import rospy
from robohow_common_msgs.msg import GraspStability
from robohow_common_msgs.srv import *
import gtk
import pygtk
import thread
import threading


class RobotControlUI:
    def __init__(self):
        rospy.init_node('cram_robot');
        # self.pubState = rospy.Publisher('/grasp_stability_estimator/state', GraspStability, latch=True);
        # self.srvControl = rospy.Service('/grasp_stability_estimator/control', GraspStabilityControl, self.control_callback)
        
        # The window
        self.window = gtk.Window(gtk.WINDOW_TOPLEVEL)
        self.window.set_keep_above(True)
        self.window.connect("delete_event", self.delete_event)
        self.window.connect("destroy", self.destroy)
        
        # The quit button
        self.button_quit = gtk.Button("Quit")
        self.button_quit.connect("clicked", self.click_quit, None)

        # The start button
        self.button_start = gtk.Button("Start Task")
        self.button_start.connect("clicked", self.click_start, None)
        self.button_start.set_sensitive(False)
        
        # The stop button
        self.button_stop = gtk.Button("Stop Task")
        self.button_stop.connect("clicked", self.click_stop, None)
        self.button_stop.set_sensitive(False)
        
        # The reset button
        self.button_reset = gtk.Button("Reset")
        self.button_reset.connect("clicked", self.click_reset, None)
        
        # The context list
        self.list_store = gtk.ListStore(str)
        self.context_list = gtk.TreeView(self.list_store)
        col_contextid = gtk.TreeViewColumn('Context ID')
        col_taskname = gtk.TreeViewColumn('Task Name')
        col_objectid = gtk.TreeViewColumn('Object ID')
        col_minstab = gtk.TreeViewColumn('Minimal Stability')
        cell = gtk.CellRendererText()
        self.context_list.append_column(col_contextid)
        self.context_list.append_column(col_taskname)
        self.context_list.append_column(col_objectid)
        self.context_list.append_column(col_minstab)
        col_contextid.pack_start(cell, 0)
        col_taskname.pack_start(cell, 0)
        col_objectid.pack_start(cell, 0)
        col_minstab.pack_start(cell, 0)
        
        col_contextid.set_attributes(cell, text=0)
        col_taskname.set_attributes(cell, text=0)
        col_objectid.set_attributes(cell, text=0)
        col_minstab.set_attributes(cell, text=0)
        list_selection = self.context_list.get_selection()
        list_selection.connect("changed", self.context_list_selection_changed)
        
        # The quit/start/stop/reset hbox
        self.hbox2 = gtk.HBox(False, 0)
        self.hbox2.pack_start(self.button_quit, False, False, 5)
        self.hbox2.pack_start(self.button_start, True, True, 5)
        self.hbox2.pack_start(self.button_stop, True, True, 5)
        self.hbox2.pack_start(self.button_reset, True, True, 5)
        
        # The list/quit/publish vbox
        self.vbox1 = gtk.VBox(False, 0)
        self.vbox1.pack_start(self.context_list, True, True, 5)
        self.vbox1.pack_start(self.hbox2, False, False, 5)
        
        # Task name
        self.hbox_taskname = gtk.HBox(False, 0)
        self.lbl_taskname = gtk.Label("Task Name: ")
        self.txt_taskname = gtk.Entry(0)
        self.hbox_taskname.pack_start(self.lbl_taskname, False, False, 5)
        self.hbox_taskname.pack_start(self.txt_taskname, False, False, 5)
        self.txt_taskname.set_text('grasp')
        self.txt_taskname.set_width_chars(15)
        
        # Object ID
        self.hbox_objectid = gtk.HBox(False, 0)
        self.lbl_objectid = gtk.Label("Object ID: ")
        self.txt_objectid = gtk.Entry(0)
        self.hbox_objectid.pack_start(self.lbl_objectid, False, False, 5)
        self.hbox_objectid.pack_start(self.txt_objectid, False, False, 5)
        self.txt_objectid.set_text('mug_1')
        self.txt_objectid.set_width_chars(15)
        
        # Context ID
        self.hbox_contextid = gtk.HBox(False, 0)
        self.lbl_contextid = gtk.Label("Context ID: ")
        self.txt_contextid = gtk.Entry(0)
        self.hbox_contextid.pack_start(self.lbl_contextid, False, False, 5)
        self.hbox_contextid.pack_start(self.txt_contextid, False, False, 5)
        self.txt_contextid.set_text('left_gripper')
        self.txt_contextid.set_width_chars(15)
        
        # Minimum Grasp quality
        self.hbox_qual = gtk.HBox(False, 0)
        self.lbl_graspqual = gtk.Label("Minimum Stability: ")
        self.txt_graspqual = gtk.Entry(0)
        self.hbox_qual.pack_start(self.lbl_graspqual, False, False, 5)
        self.hbox_qual.pack_start(self.txt_graspqual, False, False, 5)
        self.txt_graspqual.set_text('0.73')
        self.txt_graspqual.set_width_chars(5)
        
        # The settings vbox
        self.vbox2 = gtk.VBox(False, 0)
        self.vbox2.pack_start(self.hbox_taskname, False, False, 5)
        self.vbox2.pack_start(self.hbox_objectid, False, False, 5)
        self.vbox2.pack_start(self.hbox_contextid, False, False, 5)
        self.vbox2.pack_start(self.hbox_qual, False, False, 5)
        
        # The main hbox
        self.hbox1 = gtk.HBox(False, 0)
        self.hbox1.pack_start(self.vbox1, True, True, 5)
        self.hbox1.pack_start(self.vbox2, True, True, 5)
        
        self.window.add(self.hbox1)
        
        self.window.show_all()
        
        self.window.set_title("Robot Dummy UI")
        self.window.resize(500, 230)
        
        gtk.gdk.threads_init()
        thread.start_new_thread(self.spinner, ())
        
        gtk.main()
        
        pass;
    
    def spinner(self):
        rospy.spin()
    
    def control_callback(self, req):
        if req.command == 0:
            self.add_context(req.context_id)
        elif req.command == 1:
            self.remove_context(req.context_id)
        
        ctrlRet = GraspStabilityControlResponse()
        ctrlRet.result = 1;
        
        return ctrlRet
    
    def context_list_selection_changed(self, list_selection):
        (model, pathlist) = list_selection.get_selected_rows()
        value = ''
        
        if len(pathlist) > 0:
            list_iter = model.get_iter(pathlist[0])
            value = model.get_value(list_iter, 0)
        
        self.context_selected = value
        if self.context_selected == '':
            self.button_publish.set_sensitive(False)
        else:
            self.button_publish.set_sensitive(True)

    def delete_event(self, widget, event, data=None):
        return False
    
    def quit(self):
        gtk.main_quit()
    
    def click_quit(self, widget, data=None):
        self.quit()
        
    def click_reset(self, widget, data=None):
        self.list_store.clear()
        
    def click_start(self, widget, data=None):
        grasp_stability = float(self.txt_graspqual.get_text())
        context_id = self.context_selected
        
        rospy.sleep(0.1)
        self.publish(grasp_stability, context_id)

    def click_stop(self, widget, data=None):
        grasp_stability = float(self.txt_graspqual.get_text())
        context_id = self.context_selected
        
        rospy.sleep(0.1)
        self.publish(grasp_stability, context_id)
    
    def add_context(self, context):
        self.context_list.get_model().append([context])
    
    def remove_context(self, context):
        for row in self.list_store:
            if row[0] == context:
                self.list_store.remove(row.iter)
                break
    
    def destroy(self, widget, data=None):
        self.quit()
    
    def publish(self, stability, context):
        gsPublish = GraspStability()
        gsPublish.stability = stability
        gsPublish.context_id = context
        
        self.pubState.publish(gsPublish)


if __name__ == '__main__':
    rcuMain = RobotControlUI()
