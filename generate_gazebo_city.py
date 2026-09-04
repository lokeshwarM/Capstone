import os
import sys

# Re-export and execute the enhanced city generation
from generate_new_city import generate_enhanced_city_sdf

if __name__ == "__main__":
    out_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "city.sdf")
    generate_enhanced_city_sdf(out_file)
