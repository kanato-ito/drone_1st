/**
 * @file cmd_msg.h
 * @brief コマンドキュー(コア1→0)とその返信(コア0→1)のメッセージ型。
 *
 * パラメータ変更・方策の重み・制御器の選択。コア0がループの最初に処理する（設計メモ4章）。
 * 返信(受付/拒否, 設計メモ6章)は4章の表に経路が無いので、返信キューを提案する
 * （architecture.md #7）。方策の重み約21KBは断片に分けて送る（提案）。
 */
#ifndef CMD_MSG_H
#define CMD_MSG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CMD_CHUNK_FLOATS 48  /**< 断片あたりのfloat数（提案）。重みは約113断片 */

typedef enum {
    CMD_NONE = 0,
    CMD_SET_PARAM,
    CMD_SELECT_CONTROLLER,
    CMD_POLICY_CHUNK,
    CMD_POLICY_COMMIT
} cmd_type_t;

typedef struct {
    uint8_t  type;        /**< cmd_type_t */
    uint8_t  reserved;
    uint16_t param_id;    /**< SET_PARAM */
    uint32_t request_id;  /**< 返信と対応づける */
    union {
        float value;                                   /**< SET_PARAM */
        uint32_t controller_id;                        /**< SELECT_CONTROLLER */
        struct { uint32_t offset_floats; uint32_t count; float data[CMD_CHUNK_FLOATS]; } chunk; /**< POLICY_CHUNK */
        struct { uint32_t crc32; uint32_t activation; } commit; /**< POLICY_COMMIT */
    } u;
} cmd_msg_t;

typedef enum {
    CMD_RESULT_ACCEPTED = 0,
    CMD_RESULT_REJECTED_STATE,    /**< 飛行指示待機でない */
    CMD_RESULT_REJECTED_INVALID,  /**< 範囲外・CRC不一致など */
    CMD_RESULT_REJECTED_BUSY
} cmd_result_t;

typedef struct {
    uint32_t request_id;
    uint8_t  result;      /**< cmd_result_t */
    uint8_t  reserved[3];
} cmd_reply_t;

#ifdef __cplusplus
}
#endif

#endif /* CMD_MSG_H */
