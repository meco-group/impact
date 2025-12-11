from lxml import etree

class Mask:
  def __init__(self,s_function, port_labels_in=None, port_labels_out=None, block_name="", name=None, port_in=None,port_out=None,dependencies=None,init_code=""):
    self.dependencies = dependencies
    self.s_function = s_function
    self.port_labels_in = port_labels_in
    self.port_labels_out = port_labels_out
    if port_in is None:
      port_in = []
    self.port_in = port_in
    self.init_code = init_code
    self.name = name
    self.block_name = block_name
    self.stride_height  = 40
    self.padding_height = 10
    self.unit_height  = 40

    self.margin_left = 100
    self.margin_top = 100

    self.constant_width = 30
    self.constant_height = 30

    self.line_width = 100
    self.sfun_width = 200
    self.sfun_height = self.stride_height*max(len(self.port_labels_in),len(self.port_labels_out))

    self.constant_height_offset = 5
    self.zorder = 7

    mask_commands = []
    for k,e in enumerate(self.port_labels_in):
      mask_commands.append(f"port_label('input',{k+1},'{e}');")
    mask_commands.append(f"disp('{self.name}');")
    for k,e in enumerate(self.port_labels_out):
      mask_commands.append(f"port_label('output',{k+1},'{e}');")

    self.mask_commands = "\n".join(mask_commands)
  def register(self, base):
    num_elem = len(self.port_in)+2
    self.base_id = base
    self.max_id = self.base_id+num_elem

  @property
  def width(self):
    return self.sfun_width+self.constant_width+self.line_width
  @property
  def height(self):
    return self.sfun_height

  def write(self,system,offset_left=0,offset_top=0):
    margin_left = self.margin_left+offset_left
    margin_top = self.margin_top+offset_top

    sfun_margin_left = margin_left+self.constant_width+self.line_width
    sfun_margin_top = margin_top


    s = f"""
    <Block BlockType="S-Function" Name="Block_{self.block_name}" SID="{self.base_id}">
      <P Name="Ports">[{len(self.port_labels_in)}, {len(self.port_labels_out)}]</P>
      <P Name="Position">[{sfun_margin_left}, {sfun_margin_top}, {sfun_margin_left+self.sfun_width}, {sfun_margin_top+self.sfun_height}]</P>
      <P Name="ZOrder">{self.zorder}</P>
      <P Name="FunctionName">{self.s_function}</P>
      <P Name="SFunctionDeploymentMode">off</P>
      <P Name="EnableBusSupport">off</P>
      <P Name="SFcnIsStateOwnerBlock">off</P>
      <P Name="InitFcn">{self.init_code}</P>
      <Object PropName="MaskObject" ObjectID="{self.base_id+7}" ClassName="Simulink.Mask">
        <P Name="Display" Class="char">{self.mask_commands}</P>
      </Object>"""
    if self.dependencies:
      s+=f"""<P Name="SFunctionModules">&apos;{" ".join(self.dependencies)}&apos;</P>"""
    s += f"</Block>"
    try:
      system.append(etree.fromstring(s))
    except Exception as e:
      for i,line in enumerate(s.split("\n")):
        print(i,line)
      raise e

    for i, (default, label) in enumerate(zip(self.port_in,self.port_labels_in)):
      id = self.base_id+i+2
      if default is None: continue
      zorder = 7
      block = etree.fromstring(f"""
      <Block BlockType="Constant" Name="Constant{id}" SID="{id}">
        <P Name="Position">[{margin_left}, {margin_top+i*self.stride_height+self.constant_height_offset}, {margin_left+self.constant_width}, {margin_top+self.constant_height+i*self.stride_height+self.constant_height_offset}]</P>
        <P Name="ZOrder">{zorder}</P>
        <P Name="Value">{default}</P>
        <P Name="VectorParams1D">off</P>
      </Block>
      """)
      system.append(block)

      block = etree.fromstring(f"""
      <Line>
        <P Name="ZOrder">1</P>
        <P Name="Src">{id}#out:1</P>
        <P Name="Dst">{self.base_id}#in:{i+1}</P>
      </Line>
      """)
      system.append(block)

