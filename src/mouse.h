/**
 * @brief Describes mouse buttons status.
 */
struct MouseButtons {
  uint8_t left   : 1;   /**< Contains 1 when left button is pressed. */
  uint8_t middle : 1;   /**< Contains 1 when middle button is pressed. */
  uint8_t right  : 1;   /**< Contains 1 when right button is pressed. */

  MouseButtons() : left(0), middle(0), right(0) { }
};

/**
 * @brief Describes mouse movement and buttons status.
 */
struct MouseDelta {
  int16_t      deltaX;             /**< Horizontal movement since last report. Moving to the right generates positive values. */
  int16_t      deltaY;             /**< Vertical movement since last report. Upward movement generates positive values. */
  int8_t       deltaZ;             /**< Scroll wheel movement since last report. Downward movement genrates positive values. */
  MouseButtons buttons;            /**< Mouse buttons status. */
  uint8_t      overflowX : 1;      /**< Contains 1 when horizontal overflow has been detected. */
  uint8_t      overflowY : 1;      /**< Contains 1 when vertical overflow has been detected. */
};

bool mouseDeltaAvailable(void);
bool getNextMouseDelta(MouseDelta* delta);