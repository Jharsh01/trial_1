import argparse
import yaml

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("param", help="input file containing map and obstacles")
    parser.add_argument("output", help="output file with the schedule")
    args = parser.parse_args()
   
    with open(args.param, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimension = param["map"]["dimensions"]
    nodes = param["map"]["nodes"]
    agents = param['agents']
    vertex_data = {node['vertex']: node['edge'] for node in nodes}
    print (vertex_data[(1,1)])
   
    

if __name__ == "__main__":
    main()