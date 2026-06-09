/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file
 * @brief Model handler
 */

#ifndef MODEL_HANDLER_H__
#define MODEL_HANDLER_H__

#include <zephyr/bluetooth/mesh.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BLE Mesh로 수신한 원격 노드 가속도 데이터.
 *
 * x/y/z_centi 는 실제값 × 100 의 정수 표현.
 * 예) x_centi = 123  →  1.23 m/s²
 *     y_centi = -234 → -2.34 m/s²
 */
struct model_handler_remote_accel {
	uint16_t addr;     /**< 송신 노드 유니캐스트 주소 */
	int32_t  x_centi; /**< X축 가속도 × 100 */
	int32_t  y_centi; /**< Y축 가속도 × 100 */
	int32_t  z_centi; /**< Z축 가속도 × 100 */
	bool     valid;   /**< 첫 수신 이후 true */
};

/** Mesh composition data 초기화 및 반환. main()에서 bt_mesh_init()에 전달. */
const struct bt_mesh_comp *model_handler_init(void);

/**
 * @brief 로컬 가속도 데이터를 BLE Mesh로 publish.
 *
 * 노드가 provisioned 된 이후에만 실제 전송이 이루어진다.
 * 값은 sensor_value 기준 centiunits (val1×100 + val2/10000).
 */
void model_handler_publish_accel(int32_t x_centi, int32_t y_centi, int32_t z_centi);

/**
 * @brief 마지막으로 수신한 원격 노드 가속도 데이터 반환.
 *
 * valid 필드가 false이면 아직 수신된 데이터가 없음.
 */
const struct model_handler_remote_accel *model_handler_get_remote_accel(void);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_HANDLER_H__ */
