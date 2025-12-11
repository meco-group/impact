import os
import shutil
from zipfile import ZipFile 
from lxml import etree


class Diagram:
  def __init__(self,template,simulink_library_name,simulink_library_dirname=None):
    self.masks = []
    self.template = template
    self.simulink_library_name = simulink_library_name
    self.simulink_library_dirname = simulink_library_dirname
    self.simulink_library_filename = simulink_library_dirname+".slx"
    self.next_id = 1

  def add(self,mask):
    mask.register(self.next_id)
    self.next_id = mask.max_id
    self.masks.append(mask)

  def write(self):
    with ZipFile(self.template) as zipfile:
      zipfile.extractall(path=self.simulink_library_dirname)
    blockdiagram_filename = os.path.join(self.simulink_library_dirname,"simulink","blockdiagram.xml")
    with open(blockdiagram_filename,'r') as blockdiagram_file:
      tree = etree.parse(blockdiagram_file)

    system = tree.find('.//System')
    for e in system.findall('Block'):
      system.remove(e)
    for e in system.findall('Line'):
      system.remove(e)


    height_spacing = 100
    max_width = 1920
    offset_x = 0
    offset_y = 0
    height = 0
    max_id = 0
    for m in self.masks:
      # Overflow
      if offset_x+m.width>max_width and offset_x>0:
        offset_y += height+height_spacing
        offset_x = 0
      else:
        offset_x += m.width
        height = max(height, m.height)
      m.write(system,offset_left = offset_x, offset_top=offset_y)
      max_id = max(max_id, m.max_id)

    highwater = system.find('P[@Name="SIDHighWatermark"]')
    highwater.text = str(max_id)

    tree.write(blockdiagram_filename)
    shutil.make_archive(self.simulink_library_filename,'zip',self.simulink_library_dirname)
    shutil.move(self.simulink_library_filename+".zip",self.simulink_library_filename)
