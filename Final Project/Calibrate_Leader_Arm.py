from lerobot.teleoperators.so_leader import SO101LeaderConfig, SO101Leader
config = SO101LeaderConfig(
port="com8", # <- replace with YOUR leader port
id="my_awesome_leader_arm",
)
leader = SO101Leader(config)
leader.connect(calibrate=False)
leader.calibrate()
leader.disconnect()