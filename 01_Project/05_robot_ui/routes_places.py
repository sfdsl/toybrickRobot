"""Named map places. Writes are authenticated by the shared HTTP router."""
from robot_ui.places import PlaceError


def _arg(q, key):
    return q.get(key, [''])[0]


def listing(h, q):
    try:
        h._send_json({'ok': True, **h.node.places.list()})
    except (PlaceError, OSError, ValueError) as e:
        h._send_json({'ok': False, 'msg': str(e)})


def add(h, q):
    try:
        place = h.node.places.add(_arg(q, 'name'), _arg(q, 'x'), _arg(q, 'y'))
        h._send_json({'ok': True, 'place': place})
    except (PlaceError, OSError, ValueError) as e:
        h._send_json({'ok': False, 'msg': str(e)})


def delete(h, q):
    try:
        h.node.places.delete(_arg(q, 'name'))
        h._send_json({'ok': True})
    except (PlaceError, OSError, ValueError) as e:
        h._send_json({'ok': False, 'msg': str(e)})


def go(h, q):
    try:
        place, map_id = h.node.places.get(_arg(q, 'name'))
        requested = _arg(q, 'map_id')
        if requested and requested != map_id:
            raise PlaceError('地图版本已变，请刷新地点列表')
        ok, msg = h.node.modes.claim('web-nav', 'NAV', force=True, force_unless='PATROL')
        if not ok:
            raise PlaceError(msg)
        h._send_json(h.node.set_goal(x=place['x'], y=place['y']))
    except (PlaceError, OSError, ValueError) as e:
        h._send_json({'ok': False, 'msg': str(e)})
