import os
import json
import rospkg

from datetime import datetime

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

def verify_path(path, is_in_package=True):
    """Verifies the path

    If the path is relative to the SCF4 package, it is appended to the
    absolute directory of the SCF4 package. It also checks whether the
    path itself exists and if it does not, it creates the corresponding
    directory(-ies) contained within the path.

    Args:
        path (str): The desired directory or path to file
        is_in_package (bool, optional): Whether the relative path is in
            package. Defaults to True.

    Returns:
        str: An existing verified path
    """
    if is_in_package:
        # Change the relative path to the package directory
        rel_path = rospkg.RosPack().get_path("scf4_control")
        path = os.path.join(rel_path, path)
    
    if not os.path.exists(path):
        # Get the non-existing full path to the directory and create it 
        dirname = path if os.path.isdir(path) else os.path.dirname(path)
        os.makedirs(dirname)
    
    return path

def parse_json(path, is_in_package=True):
    """Reads a json file and converts to python dictionary

    Simply takes the location of a json file, opens it in read mode and
    extracts its contents to a python dictionary.

    Args:
        path (str): The path to the json file
        is_in_package (bool, optional): Whether the relative path is in
            package. Defaults to True.

    Returns:
        dict: A parsed json dictionary
    """
    # Verify the path to the given JSON file
    path = verify_path(path, is_in_package)

    with open(path, 'r') as f:
        # Parse to python dict
        data = json.load(f)
    
    return data

def get_fourcc(fourcc):
    """Converts integer FOURCC to character code

    Takes an integer value which is converted to hexadecimal number
    which can be decoded into a string name.

    Note: https://stackoverflow.com/a/71838016

    Args:
        fourcc_code (int): The integer encoding the 4-character code

    Returns:
        str: A decoded FOURCC
    """
    fourcc_name = bytes([
        v & 255 for v in (fourcc, fourcc >> 8, fourcc >> 16, fourcc >> 24)
    ]).decode()
    
    return fourcc_name

def get_str_datetime(format="%d-%m-%Y_%H.%M.%S"):
    now = datetime.now()
    str_now = now.strftime(format)

    return str_now

def clamp(val, min_val, max_val, inv_range=False):
    if inv_range and (val < 0 < min_val or val > 0 > max_val):
        # If inverse range is included, swap
        min_val, max_val = -max_val, -min_val
    
    return min(max(val, min_val), max_val)