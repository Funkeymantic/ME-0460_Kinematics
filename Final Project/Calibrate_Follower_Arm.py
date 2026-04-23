from lerobot.robots.so_follower import SO101FollowerConfig, SO101Follower
config = SO101FollowerConfig(
port="com6", # <- replace with YOUR follower port
id="my_awesome_follower_arm",
)
follower = SO101Follower(config)
follower.connect(calibrate=False)
follower.calibrate()
follower.disconnect()