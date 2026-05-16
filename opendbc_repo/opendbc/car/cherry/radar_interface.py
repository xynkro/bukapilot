from opendbc.car.interfaces import RadarInterfaceBase


class RadarInterface(RadarInterfaceBase):
  """Cherry PT DBC has no front-radar track messages; use default no-op radar."""
