from geometry_msgs.msg import Vector3

def adc_to_volt(adc, v_ref=3.3, resolution=4096, scale=0.5, parse=True):
    """Converts ADC value to voltage

    Takes the digitalized current (voltage) and converts it to analog
    based on resolution and resistance factor.

    Args:
        adc (str|float): The digitalized analog signal (voltage) value
        v_ref (float, optional): The reference voltage between which and
            `0` the analog signal varies. Defaults to 3.3.
        resolution (int, optional): The number of bits representing the
            current. For SCF4, it is 12 bit. Defaults to 4096.
        scale (float, optional): The resistance divider the control
            board uses. Defaults to 0.5.
        parse (bool, optional): Whether to parse the ADC input to float
            given it comes in the format "ADC={val}". Defaults to True.

    Returns:
        float: The voltage the SCF4 module receives
    """
    # Parse if needed and compute voltage
    adc = float(adc[4:]) if parse else adc
    v_in = (adc / resolution) * (v_ref / scale)

    return v_in