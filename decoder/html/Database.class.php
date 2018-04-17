<?php
require_once "Tracker.class.php";

class Database extends mysqli {
	private static $instance = null;

	private function __construct() {
		$this->con = parent::__construct("localhost", "decoder", "decoder", "decoder");
		if(mysqli_connect_errno())
			die(mysql_error());
	}

	public function __destruct() {
		$this->close();
	}

	public static function getInstance() {
		if(self::$instance == null)
			self::$instance = new Database();
		return self::$instance;
	}

	public function close() {
		if(is_null($this->con))
			return;
		parent::close();
		$this->con = null;
	}

	public function getTracker() {
		$tracker = array();

		$query = $this->query("
			SELECT `call`,MAX(`rxtime`)
			FROM (
				SELECT `call`,`rxtime` FROM `position`
				UNION ALL
				SELECT `call`,`rxtime` FROM `image`
			) AS d
			GROUP BY `call`
			ORDER BY `rxtime` DESC
		");
		while($row = $query->fetch_assoc())
			$tracker[] = new Tracker($row['call']);

		return $tracker;
	}
}
?>

