import cv2
import numpy as np
import os


def create_aruco_marker_with_border(marker_id=0, marker_size=1000, border_size=50,
                                    aruco_dict_type=cv2.aruco.DICT_6X6_250, save_path='marker_image.jpg'):
    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    marker_image = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size)

    # Создание отступов вокруг маркера
    bordered_marker_image = cv2.copyMakeBorder(marker_image, border_size, border_size, border_size, border_size,
                                               cv2.BORDER_CONSTANT, value=[255, 255, 255])

    cv2.imwrite(save_path, bordered_marker_image)
    return os.path.abspath(save_path)


def detect_aruco_marker(image_path, aruco_dict_type=cv2.aruco.DICT_5X5_250):
    # Загрузка изображения
    image = cv2.imread(image_path)

    # Проверка, что изображение было успешно загружено
    if image is None:
        print(f"Could not open or find the image: {image_path}")
        return

    # Получение словаря ArUco и параметров детектора
    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    aruco_params = cv2.aruco.DetectorParameters_create()

    # Детекция маркеров в изображении
    corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=aruco_params)

    # Отображение результатов
    if ids is not None:
        for i, corner in enumerate(corners):
            # Обрамление детектированных маркеров в изображении
            cv2.polylines(image, [np.int32(corner)], True, (0, 255, 0), 2)
            cv2.putText(image, str(ids[i][0]), tuple(np.int32(corner[0][0])), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2)
        print(f"Detected ArUco markers: {','.join(str(id[0]) for id in ids)}")
    else:
        print("No ArUco markers detected")

    # Показ изображения с детектированными маркерами
    cv2.imshow('ArUco Detection', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# Пример использования тестовой функции
if __name__ == "__main__":
    # filepath = create_aruco_marker_with_border(aruco_dict_type=cv2.aruco.DICT_6X6_250)
    filepath = "/home/vadim/Desktop/image.png"
    detect_aruco_marker(filepath, aruco_dict_type=cv2.aruco.DICT_6X6_250)