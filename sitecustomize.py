import os
import sys


def _add_user_site_packages() -> None:
    user_site = os.path.join(
        os.environ.get("APPDATA", ""),
        "Python",
        f"Python{sys.version_info.major}{sys.version_info.minor}",
        "site-packages",
    )
    if user_site and os.path.isdir(user_site) and user_site not in sys.path:
        sys.path.append(user_site)


_add_user_site_packages()
