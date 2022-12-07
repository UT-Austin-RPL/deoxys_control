import cv2


def resize_img(img, camera_type, img_w=128, img_h=128, offset_w=0, offset_h=0):

    if camera_type == "k4a":
        resized_img = cv2.resize(img, (0, 0), fx=0.2, fy=0.2)
        w = resized_img.shape[0]
        h = resized_img.shape[1]

    if camera_type == "rs":
        resized_img = cv2.resize(img, (0, 0), fx=0.2, fy=0.3)
        w = resized_img.shape[0]
        h = resized_img.shape[1]

    resized_img = resized_img[
        w // 2 - img_w // 2 : w // 2 + img_w // 2,
        h // 2 - img_h // 2 : h // 2 + img_h // 2,
        :,
    ]
    return resized_img
