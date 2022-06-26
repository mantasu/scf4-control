import os
import json
import rospkg

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
        float: The voltage the SCF4 module receives. Might be `None` if
            the provided `adc` is an empty string
    """
    if isinstance(adc, str):
        # Convert to proper type
        if adc == "": return None
        if parse: adc = adc[4:]
        adc = float(adc)
    
    # Compute voltage given the ADC voltage
    v_in = (adc / resolution) * (v_ref / scale)

    return v_in

def parse_json(path, is_in_package=True):
    """Reads a json file and converts to python dictionary

    Simply takes the location of a json file, opens it in read mode and
    extracts its contents to a python dictionary.

    Args:
        path (str): The path to the json file
        is_in_package (bool): Whether the relative path is in package

    Returns:
        dict: A parsed json dictionary
    """
    if is_in_package:
        # Change the relative path to the package directory
        rel_path = rospkg.RosPack().get_path("scf4_control")
        path = os.path.join(rel_path, path)

    with open(path, 'r') as f:
        # Parse to python dict
        data = json.load(f)
    
    return data