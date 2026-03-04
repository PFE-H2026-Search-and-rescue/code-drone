import time

import numpy as np
import numpy.typing as npt

import pyvista as pv
from pyvista import examples, Plotter
from trame.app import get_server
from trame_server import Server
from trame_server.controller import Controller
from trame_server.state import State
from pyvista.trame.ui import plotter_ui
from trame.ui.vuetify3 import SinglePageLayout


class Web_Viewer():

    server : Server
    pl : Plotter

    def __init__(self):
        pv.OFF_SCREEN = True

        self.server = get_server()
        state, ctrl = self.server.state, self.server.controller

        self.pl = pv.Plotter()


        with SinglePageLayout(self.server) as layout:
            with layout.content:
                # Use PyVista's Trame UI helper method
                #  this will add UI controls
                view = plotter_ui(self.pl)





    def start_server(self):
        self.server.start(port=8886, host='0.0.0.0', timeout=0, open_browser=False)

    def add_points(self, point_array : npt.NDArray):
        print("Setting points")
        # self.pl.clear()
        self.pl.add_points(point_array)
