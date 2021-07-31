from Tkinter import *
import pdb
from bot_overseer_api import OverseerAPI

class DialogBox(object):

    def __init__(self):
       self.state = "UNINIT"

    def dialog_box(self, message="empty"):
    
        self.state = "ACTIVE"
        self.main = Tk()
        mg = Message(self.main, text=message)
        mg.pack()

        button = Button(self.main, text='Ok', width=25, command=self.destroy) 
        button.pack()

        self.main.mainloop()
    
    def destroy(self):
        self.state = "DONE"
        self.main.destroy()
        del self.main
    
    def get_state(self):
        return self.state

if __name__ == "__main__":
    
    db = DialogBox()
    op = OverseerAPI()
    pdb.set_trace()