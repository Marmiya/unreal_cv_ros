import unrealcv_api
import unrealcv
import cv2

client = unrealcv_api.UnrealCv_API(9000, '127.0.0.1', (1024, 768), mode="unix")


print(client.client.request('vget /cameras'))

print(client.get_cam_location(0))

client.set_cam_location(0,(0,0,1500))

image = client.get_image(1, 'lit', 'png')

print(client.get_cam_location(0))

# save the image to disk
cv2.imwrite('/home/marinaio/catkin_ws/mlogs/test.png', image)