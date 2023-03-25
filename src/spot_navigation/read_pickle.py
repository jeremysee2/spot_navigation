import pickle


def read_fiducial_pickle():
    fiducials = pickle.load(open("fiducials_seen.pickle", "rb"))
    print(type(fiducials))
    for tag_id, data in fiducials.items():
        print(tag_id)
        print(data)


if __name__ == "__main__":
    read_fiducial_pickle()
